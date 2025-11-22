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
    private let rayGridSize = 5            // cast rayGridSize x rayGridSize rays in a square
    private let rayScreenRadius: CGFloat = 0.12 // fraction of view min(width,height) to scan (e.g. 0.12)
    private let maxRayDistance: Float = 3.0 // meters
    private let minRayDistance: Float = 0.15
    private let featurePointConeHalfWidth: Float = 0.25 // meters left/right tolerance in camera space
    private let featurePointConeHalfHeight: Float = 0.25
    private let featurePointNearZ: Float = -0.2
    private let featurePointFarZ: Float = -2.5
    private let featurePointDensityThreshold = 60       // points counted inside cone -> treat as obstacle
    private let densityFallbackMinDistance: Float = 0.4 // if density triggers, treat obstacle at this conservative distance

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

        // 2. Obstacle detection (raycast primary, feature-point density fallback)
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

    /// Returns the estimated obstacle distance in meters.
    /// If no obstacle is detected within maxRayDistance, returns a large value (e.g. 10.0).
    private func detectObstacleDistance(frame: ARFrame) -> Float {
        // 1) Try multi-raycast over a small cone around screen center.
        if let rayDistance = performMultiRaycast() {
            return rayDistance
        }

        // 2) Raycast missed — use feature-point density fallback
        if let densityDistance = featurePointDensityFallback(frame: frame) {
            return densityDistance
        }

        // 3) Nothing detected
        return 10.0 // no obstacle nearby
    }

    /// Shoot multiple rays in a square grid around the screen center and return the nearest hit distance (meters).
    /// Returns nil if no ray hit within maxRayDistance.
    private func performMultiRaycast() -> Float? {
        guard let view = sceneView else { return nil }

        // center + grid offsets in view space (pixels)
        let bounds = view.bounds
        let center = CGPoint(x: bounds.midX, y: bounds.midY)
        let minSide = min(bounds.width, bounds.height)
        let radiusPx = rayScreenRadius * minSide

        var nearestDistance: Float? = nil

        // sample grid from -half..+half
        let half = (rayGridSize - 1) / 2
        for i in 0..<rayGridSize {
            for j in 0..<rayGridSize {
                // normalized offsets in [-1,1]
                let nx = CGFloat(i - half) / CGFloat(max(1, half))
                let ny = CGFloat(j - half) / CGFloat(max(1, half))

                // map to pixel offset within radius
                let samplePoint = CGPoint(x: center.x + nx * radiusPx,
                                          y: center.y + ny * radiusPx)

                // create raycast query from that screen point
                if let query = sceneView.raycastQuery(from: samplePoint,
                                                      allowing: .estimatedPlane,
                                                      alignment: .any) {
                    let results = sceneView.session.raycast(query)
                    if let hit = results.first {
                        // ARRaycastResult.distance is meters from camera
                        let d = Float(hit.distance)
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

    /// Fallback: counts feature points inside a forward cone in camera space.
    /// If density exceeds threshold, return a conservative distance. Otherwise nil.
    private func featurePointDensityFallback(frame: ARFrame) -> Float? {
        guard let points = frame.rawFeaturePoints?.points else { return nil }

        // convert world -> camera once
        let cameraTransform = frame.camera.transform
        let worldToCamera = cameraTransform.inverse

        var count = 0
        var nearestZ: Float? = nil

        for p in points {
            // world point as homogenous
            let worldPoint = simd_float4(p.x, p.y, p.z, 1)
            let local = simd_mul(worldToCamera, worldPoint) // camera space

            // camera space: forward is negative z
            if local.z < featurePointNearZ && local.z > featurePointFarZ {
                if abs(local.x) <= featurePointConeHalfWidth && abs(local.y) <= featurePointConeHalfHeight {
                    count += 1
                    // track nearest forward absolute distance
                    if nearestZ == nil || abs(local.z) < nearestZ! {
                        nearestZ = abs(local.z)
                    }
                }
            }
        }

        if count >= featurePointDensityThreshold {
            // if we have many points, consider there is an obstacle.
            // Use nearestZ if available, or a conservative fallback distance.
            if let nz = nearestZ {
                // clamp to reasonable bounds
                let clamped = max(minRayDistance, min(maxRayDistance, nz))
                return clamped
            } else {
                return densityFallbackMinDistance
            }
        } else {
            return nil
        }
    }
}
