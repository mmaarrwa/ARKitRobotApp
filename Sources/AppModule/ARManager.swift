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
    private let rayScreenRadius: CGFloat = 0.15 // Spans 15% of the screen center
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
        
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
            print("🟢 LiDAR Active: App Opened with LiDAR.")
        } else {
            print("🟡 Standard VIO Active: App Opened without LiDAR.")
        }
        
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
            
            if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
                config.sceneReconstruction = .mesh
                print("🟢 LiDAR Active: Streaming Started with LiDAR.")
            } else {
                print("🟡 Standard VIO Active: Streaming Started without LiDAR.")
            }
            
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

        // 2. Calculate Obstacle Distance & Get Method
        let (obstacleDistance, distanceMethod) = detectObstacleDistance(frame: frame)

        // ---------------------------------------------------------
        // PACKET 1: NAVIGATION (Sent Every Frame)
        // ---------------------------------------------------------
        let navPacket: [String: Any] = [
            "type": "nav",
            "timestamp": frame.timestamp,
            "position": [currentPos.x, currentPos.y, currentPos.z],
            "orientation": [q.vector.x, q.vector.y, q.vector.z, q.vector.w],
            "obstacle_dist": obstacleDistance,
            "obstacle_method": distanceMethod 
        ]
        network.sendPose(navPacket)

        // ---------------------------------------------------------
        // PACKET 2: SURVEYING (Sent Every 2 Meters)
        // ---------------------------------------------------------
        let distMoved = distance(currentPos, lastSurveyPosition)

        if distMoved >= surveyInterval {
            let surveyPacket: [String: Any] = [
                "type": "survey",
                "timestamp": frame.timestamp,
                "position": [currentPos.x, currentPos.y, currentPos.z],
                "label": "Survey Point", 
                "note": "Captured at \(String(format: "%.2f", distMoved))m interval"
            ]
            
            network.sendPose(surveyPacket)
            lastSurveyPosition = currentPos
            print("📍 Survey Packet Sent at: \(currentPos)")
        }
    }

    // MARK: - Obstacle Cascade Logic

    private func detectObstacleDistance(frame: ARFrame) -> (Float, String) {
        var rawDistance: Float = 10.0 // Default: No obstacle
        var activeMethod = "None"

        // [M1] Priority: LiDAR Depth Grid
        if let lidarDist = m1_lidarGrid(frame: frame) {
            rawDistance = lidarDist
            activeMethod = "M1_LiDAR"
        } 
        // [M2] Fallback 1: ARKit Raycast Grid (Modern)
        else if let rayDist = m2_raycastGrid(frame: frame) {
            rawDistance = rayDist
            activeMethod = "M2_Raycast"
        } 
        // [M3] Fallback 2: Old HitTest Grid (Legacy)
        else if let hitDist = m3_hitTestGrid(frame: frame) {
            rawDistance = hitDist
            activeMethod = "M3_HitTest"
        }
        // [M4] Fallback 3: Feature Point Cone
        else if let coneDist = m4_coneFallback(frame: frame) {
            rawDistance = coneDist
            activeMethod = "M4_Cone"
        }

        // Apply EWMA Smoothing to prevent jumping
        smoothedObstacleDist = (smoothingAlpha * rawDistance) + ((1.0 - smoothingAlpha) * smoothedObstacleDist)
        
        return (smoothedObstacleDist, activeMethod)
    }

    // MARK: - Individual Methods

    private func m1_lidarGrid(frame: ARFrame) -> Float? {
        guard let depthData = frame.smoothedSceneDepth ?? frame.sceneDepth else { return nil }
        let depthMap = depthData.depthMap
        
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        let w = CVPixelBufferGetWidth(depthMap)
        let h = CVPixelBufferGetHeight(depthMap)
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return nil }
        let buf = baseAddress.assumingMemoryBound(to: Float32.self)
        
        var nearestDistance: Float? = nil
        var hitCount = 0
        let requiredHits = 2
        
        let half = (rayGridSize - 1) / 2
        for i in 0..<rayGridSize {
            for j in 0..<rayGridSize {
                let nx = Float(i - half) / Float(max(1, half))
                let ny = Float(j - half) / Float(max(1, half))
                
                // Map to center of screen (0.5, 0.5) with radius offset
                let normX = 0.5 + (nx * Float(rayScreenRadius))
                let normY = 0.5 + (ny * Float(rayScreenRadius))
                
                let px = max(0, min(Int(normX * Float(w)), w - 1))
                let py = max(0, min(Int(normY * Float(h)), h - 1))
                
                let dist = buf[py * w + px]
                
                if dist >= minRayDistance && dist <= maxRayDistance {
                    hitCount += 1
                    if let current = nearestDistance {
                        nearestDistance = min(current, dist)
                    } else {
                        nearestDistance = dist
                    }
                }
            }
        }
        
        if hitCount >= requiredHits { return nearestDistance }
        return nil
    }

    private func m2_raycastGrid(frame: ARFrame) -> Float? {
        var nearestDistance: Float? = nil
        let semaphore = DispatchSemaphore(value: 0)

        DispatchQueue.main.async {
            defer { semaphore.signal() }
            
            let view = self.sceneView
            let bounds = view.bounds
            let center = CGPoint(x: bounds.midX, y: bounds.midY)
            let minSide = min(bounds.width, bounds.height)
            let radiusPx = self.rayScreenRadius * minSide

            let camPos = SIMD3<Float>(frame.camera.transform.columns.3.x, 
                                      frame.camera.transform.columns.3.y, 
                                      frame.camera.transform.columns.3.z)
            
            var hitCount = 0
            let requiredHits = 2 
            let half = (self.rayGridSize - 1) / 2
            
            for i in 0..<self.rayGridSize {
                for j in 0..<self.rayGridSize {
                    let nx = CGFloat(i - half) / CGFloat(max(1, half))
                    let ny = CGFloat(j - half) / CGFloat(max(1, half))

                    let samplePoint = CGPoint(x: center.x + nx * radiusPx,
                                              y: center.y + ny * radiusPx)

                    guard let query = view.raycastQuery(from: samplePoint, allowing: .estimatedPlane, alignment: .any) else { continue }
                    let results = view.session.raycast(query)
                    
                    if let hit = results.first {
                        let hitPos = SIMD3<Float>(hit.worldTransform.columns.3.x, 
                                                  hit.worldTransform.columns.3.y, 
                                                  hit.worldTransform.columns.3.z)
                        
                        let d = distance(camPos, hitPos)
                        
                        if d >= self.minRayDistance && d <= self.maxRayDistance {
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

            if hitCount < requiredHits {
                nearestDistance = nil
            }
        }
        
        semaphore.wait()
        return nearestDistance
    }

    private func m3_hitTestGrid(frame: ARFrame) -> Float? {
        var nearestDistance: Float? = nil
        let semaphore = DispatchSemaphore(value: 0)

        DispatchQueue.main.async {
            defer { semaphore.signal() }
            
            let view = self.sceneView
            let bounds = view.bounds
            let center = CGPoint(x: bounds.midX, y: bounds.midY)
            let minSide = min(bounds.width, bounds.height)
            let radiusPx = self.rayScreenRadius * minSide

            let camPos = SIMD3<Float>(frame.camera.transform.columns.3.x, 
                                      frame.camera.transform.columns.3.y, 
                                      frame.camera.transform.columns.3.z)
            
            var hitCount = 0
            let requiredHits = 2 
            let half = (self.rayGridSize - 1) / 2
            
            for i in 0..<self.rayGridSize {
                for j in 0..<self.rayGridSize {
                    let nx = CGFloat(i - half) / CGFloat(max(1, half))
                    let ny = CGFloat(j - half) / CGFloat(max(1, half))

                    let samplePoint = CGPoint(x: center.x + nx * radiusPx,
                                              y: center.y + ny * radiusPx)

                    // Using the old hitTest logic as requested
                    let results = view.hitTest(samplePoint, types: [.featurePoint, .existingPlaneUsingExtent, .estimatedHorizontalPlane])
                    
                    if let hit = results.first {
                        let hitPos = SIMD3<Float>(hit.worldTransform.columns.3.x, 
                                                  hit.worldTransform.columns.3.y, 
                                                  hit.worldTransform.columns.3.z)
                        
                        let d = distance(camPos, hitPos)
                        
                        if d >= self.minRayDistance && d <= self.maxRayDistance {
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

            if hitCount < requiredHits {
                nearestDistance = nil
            }
        }
        
        semaphore.wait()
        return nearestDistance
    }

    private func m4_coneFallback(frame: ARFrame) -> Float? {
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