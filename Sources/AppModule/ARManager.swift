import Foundation
import ARKit
import SceneKit
import simd

final class ARManager: NSObject, ObservableObject, ARSessionDelegate {
    static let shared = ARManager()

    let sceneView: ARSCNView = {
        let v = ARSCNView(frame: .zero)
        v.autoenablesDefaultLighting = true
        v.debugOptions = [ARSCNDebugOptions.showFeaturePoints]
        return v
    }()

    @Published var isStreaming: Bool = false
    @Published var statusText: String = "Idle"
    @Published var serverIP: String = "192.168.1.10"

    private let network = NetworkManager.shared

    // --- Configurable parameters ---
    private let rayGridSize = 5            // 5x5 Grid
    private let rayScreenRadius: CGFloat = 0.15 // Slightly wider scan
    private let maxRayDistance: Float = 3.0
    private let minRayDistance: Float = 0.15
    
    // Feature Point Fallback Params
    private let featurePointConeHalfWidth: Float = 0.25
    private let featurePointConeHalfHeight: Float = 0.25
    private let featurePointNearZ: Float = -0.2
    private let featurePointFarZ: Float = -2.5
    private let featurePointDensityThreshold = 60
    private let densityFallbackMinDistance: Float = 0.4

    // --- Smoothing & Confidence ---
    private var smoothedObstacleDist: Float = 10.0 // Initialize with "far"
    private let smoothingAlpha: Float = 0.2 // 0.2 = Slow/Smooth, 0.8 = Fast/Jittery

    // --- Surveying Logic ---
    private var lastSurveyPosition: SIMD3<Float> = SIMD3<Float>(0, 0, 0)
    private let surveyInterval: Float = 2.0 // Meters required to trigger a survey packet

    override init() {
        super.init()
        network.onCommandReceived = { [weak self] command in
            self?.handleRemoteCommand(command)
        }
    }

    func startSessionIfNeeded() {
        guard ARWorldTrackingConfiguration.isSupported else { return }
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        sceneView.session.run(config)
        sceneView.session.delegate = self
        statusText = "Ready to Connect"
    }

    func handleRemoteCommand(_ command: String) {
        if command == "START" {
            if !isStreaming { toggleStreaming() }
        } else if command == "STOP" {
            if isStreaming { toggleStreaming() }
        }
    }

    func toggleStreaming() {
        isStreaming.toggle()

        if isStreaming {
            statusText = "Streaming..."
            smoothedObstacleDist = 10.0 // Reset smoothing on start
            lastSurveyPosition = SIMD3<Float>(0,0,0) // Reset survey logic
            
            network.start(ipAddress: serverIP)

            let config = ARWorldTrackingConfiguration()
            config.worldAlignment = .gravity
            sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])

        } else {
            statusText = "Stopped"
            network.stop()
        }
    }

    // MARK: - ARSessionDelegate
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard isStreaming else { return }

        // 1. Get Current Pose
        let cameraTransform = frame.camera.transform
        let col3 = cameraTransform.columns.3
        let currentPos = SIMD3<Float>(col3.x, col3.y, col3.z)
        let q = simd_quatf(cameraTransform)

        // 2. Calculate Obstacle Distance (Hybrid: HitTest + Smoothing)
        let obstacleDistance = detectObstacleDistance(frame: frame)

        // ---------------------------------------------------------
        // PACKET 1: NAVIGATION (Sent Every Frame)
        // ---------------------------------------------------------
        let navPacket: [String: Any] = [
            "type": "nav",
            "timestamp": frame.timestamp,
            "position": [currentPos.x, currentPos.y, currentPos.z],
            "orientation": [q.vector.x, q.vector.y, q.vector.z, q.vector.w],
            "obstacle_dist": obstacleDistance
        ]
        network.sendPose(navPacket)

        // ---------------------------------------------------------
        // PACKET 2: SURVEYING (Sent Every 2 Meters)
        // ---------------------------------------------------------
        let distMoved = distance(currentPos, lastSurveyPosition)

        if distMoved >= surveyInterval {
            // Create survey data
            let surveyPacket: [String: Any] = [
                "type": "survey",
                "timestamp": frame.timestamp,
                "position": [currentPos.x, currentPos.y, currentPos.z],
                "label": "Survey Point", // You can replace this later with ML detection
                "note": "Captured at \(String(format: "%.2f", distMoved))m interval"
            ]
            
            network.sendPose(surveyPacket)
            
            // Reset tracker
            lastSurveyPosition = currentPos
            print("📍 Survey Packet Sent at: \(currentPos)")
        }
    }

    // MARK: - Obstacle detection helpers

    private func detectObstacleDistance(frame: ARFrame) -> Float {
        var rawDistance: Float = 10.0 // Default: No obstacle

        // 1) Priority: Grid Hit Test (High Confidence)
        if let hitDist = performGridHitTest(cameraTransform: frame.camera.transform) {
            rawDistance = hitDist
        } 
        // 2) Fallback: Feature Point Density (Medium Confidence)
        else if let densityDist = featurePointDensityFallback(frame: frame) {
            rawDistance = densityDist
        }

        // 3) Apply Exponential Weighted Moving Average (EWMA) Smoothing
        smoothedObstacleDist = (smoothingAlpha * rawDistance) + ((1.0 - smoothingAlpha) * smoothedObstacleDist)
        
        return smoothedObstacleDist
    }

    private func performGridHitTest(cameraTransform: simd_float4x4) -> Float? {
        let view = sceneView
        let bounds = view.bounds
        let center = CGPoint(x: bounds.midX, y: bounds.midY)
        let minSide = min(bounds.width, bounds.height)
        let radiusPx = rayScreenRadius * minSide

        let camPos = SIMD3<Float>(cameraTransform.columns.3.x, cameraTransform.columns.3.y, cameraTransform.columns.3.z)
        
        var nearestDistance: Float? = nil
        var hitCount = 0
        let requiredHits = 2 

        let half = (rayGridSize - 1) / 2
        for i in 0..<rayGridSize {
            for j in 0..<rayGridSize {
                let nx = CGFloat(i - half) / CGFloat(max(1, half))
                let ny = CGFloat(j - half) / CGFloat(max(1, half))

                let samplePoint = CGPoint(x: center.x + nx * radiusPx,
                                          y: center.y + ny * radiusPx)

                let results = view.hitTest(samplePoint, types: [.featurePoint, .existingPlaneUsingExtent, .estimatedHorizontalPlane])
                
                if let hit = results.first {
                    let hitPos = SIMD3<Float>(hit.worldTransform.columns.3.x, 
                                              hit.worldTransform.columns.3.y, 
                                              hit.worldTransform.columns.3.z)
                    
                    let d = distance(camPos, hitPos)
                    
                    if d >= minRayDistance && d <= maxRayDistance {
                        hitCount += 1
                        if let current = nearestDistance {
                            nearestDistance = min(current, d)
                        } else {
                            nearestDistance = d
                        }
                    }
                }
            }
        }

        if hitCount >= requiredHits {
            return nearestDistance
        }
        return nil
    }

    private func featurePointDensityFallback(frame: ARFrame) -> Float? {
        guard let points = frame.rawFeaturePoints?.points else { return nil }

        let cameraTransform = frame.camera.transform
        let worldToCamera = cameraTransform.inverse

        var count = 0
        var nearestZ: Float? = nil

        for p in points {
            let worldPoint = simd_float4(p.x, p.y, p.z, 1)
            let local = simd_mul(worldToCamera, worldPoint)

            if local.z < featurePointNearZ && local.z > featurePointFarZ {
                if abs(local.x) <= featurePointConeHalfWidth && abs(local.y) <= featurePointConeHalfHeight {
                    count += 1
                    if nearestZ == nil || abs(local.z) < nearestZ! {
                        nearestZ = abs(local.z)
                    }
                }
            }
        }

        if count >= featurePointDensityThreshold {
            if let nz = nearestZ {
                return max(minRayDistance, min(maxRayDistance, nz))
            } else {
                return densityFallbackMinDistance
            }
        } else {
            return nil
        }
    }
}