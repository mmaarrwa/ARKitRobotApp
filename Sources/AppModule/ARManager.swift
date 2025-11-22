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
    private let rayGridSize = 5
    private let rayScreenRadius: CGFloat = 0.12
    private let maxRayDistance: Float = 3.0
    private let minRayDistance: Float = 0.15
    private let featurePointConeHalfWidth: Float = 0.25
    private let featurePointConeHalfHeight: Float = 0.25
    private let featurePointNearZ: Float = -0.2
    private let featurePointFarZ: Float = -2.5
    private let featurePointDensityThreshold = 60
    private let densityFallbackMinDistance: Float = 0.4

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

        // 1. Pose
        let cameraTransform = frame.camera.transform
        let col3 = cameraTransform.columns.3
        let pos = SIMD3<Float>(col3.x, col3.y, col3.z)
        let q = simd_quatf(cameraTransform)

        // 2. Obstacle detection
        let obstacleDistance = detectObstacleDistance(frame: frame)

        // 3. Build payload and send
        let pose: [String: Any] = [
            "timestamp": frame.timestamp,
            "position": [pos.x, pos.y, pos.z],
            "orientation": [q.vector.x, q.vector.y, q.vector.z, q.vector.w],
            "obstacle_dist": obstacleDistance
        ]

        network.sendPose(pose)
    }

    // MARK: - Obstacle detection helpers

    private func detectObstacleDistance(frame: ARFrame) -> Float {
        // 1) Try multi-raycast
        if let rayDistance = performMultiRaycast(cameraTransform: frame.camera.transform) {
            return rayDistance
        }

        // 2) Fallback to feature points
        if let densityDistance = featurePointDensityFallback(frame: frame) {
            return densityDistance
        }

        // 3) Nothing detected
        return 10.0
    }

    private func performMultiRaycast(cameraTransform: simd_float4x4) -> Float? {
        let view = sceneView
        let bounds = view.bounds
        let center = CGPoint(x: bounds.midX, y: bounds.midY)
        let minSide = min(bounds.width, bounds.height)
        let radiusPx = rayScreenRadius * minSide

        var nearestDistance: Float? = nil
        
        // We need the camera position to calculate distance manually
        let camPos = SIMD3<Float>(cameraTransform.columns.3.x, cameraTransform.columns.3.y, cameraTransform.columns.3.z)

        let half = (rayGridSize - 1) / 2
        for i in 0..<rayGridSize {
            for j in 0..<rayGridSize {
                let nx = CGFloat(i - half) / CGFloat(max(1, half))
                let ny = CGFloat(j - half) / CGFloat(max(1, half))

                let samplePoint = CGPoint(x: center.x + nx * radiusPx,
                                          y: center.y + ny * radiusPx)

                if let query = sceneView.raycastQuery(from: samplePoint,
                                                      allowing: .estimatedPlane,
                                                      alignment: .any) {
                    let results = sceneView.session.raycast(query)
                    if let hit = results.first {
                        // FIX: Calculate distance manually because ARRaycastResult has no .distance property
                        let hitPos = SIMD3<Float>(hit.worldTransform.columns.3.x, 
                                                  hit.worldTransform.columns.3.y, 
                                                  hit.worldTransform.columns.3.z)
                        
                        // Distance formula: √((x2-x1)^2 + ...)
                        let d = distance(camPos, hitPos)
                        
                        if d >= minRayDistance && d <= maxRayDistance {
                            if let current = nearestDistance {
                                nearestDistance = min(current, d)
                            } else {
                                nearestDistance = d
                            }
                        }
                    }
                }
            }
        }

        return nearestDistance
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