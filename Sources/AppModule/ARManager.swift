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

    // MARK: - Grid Parameters (shared by raycast and hitTest)
    private let rayGridSize          = 5              // 5×5 grid
    private let rayScreenRadius:     CGFloat = 0.15   // grid radius as fraction of screen min-side
    private let maxRayDistance:      Float   = 3.0
    private let minRayDistance:      Float   = 0.15
    private let requiredHits                 = 2      // min hits to trust result

    // MARK: - Feature Point Cone Parameters
    private let fpConeHalfWidth:     Float   = 0.25
    private let fpConeHalfHeight:    Float   = 0.25
    private let fpNearZ:             Float   = 0.20   // ignore closer than 0.2 m
    private let fpFarZ:              Float   = 2.50   // ignore farther than 2.5 m
    private let fpDensityThreshold           = 60
    private let fpFallbackMinDist:   Float   = 0.40

    // MARK: - Smoothing
    private var smoothedObstacleDist: Float  = 10.0
    private let smoothingAlpha:       Float  = 0.2    // 0.2=smooth, 0.8=reactive

    // MARK: - Surveying
    private var lastSurveyPosition: SIMD3<Float> = .zero
    private let surveyInterval:      Float   = 2.0    // metres between survey packets

    // MARK: - Init
    override init() {
        super.init()
        network.onCommandReceived = { [weak self] command in
            self?.handleRemoteCommand(command)
        }
    }

    // MARK: - Session Management
    func startSessionIfNeeded() {
        guard ARWorldTrackingConfiguration.isSupported else { return }
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
            print("🟢 LiDAR Active")
        } else {
            print("🟡 Standard VIO")
        }
        sceneView.session.run(config)
        sceneView.session.delegate = self
        statusText = "Ready to Connect"
    }

    func handleRemoteCommand(_ command: String) {
        if command == "START", !isStreaming { toggleStreaming() }
        else if command == "STOP", isStreaming  { toggleStreaming() }
    }

    func toggleStreaming() {
        isStreaming.toggle()
        if isStreaming {
            statusText            = "Streaming..."
            smoothedObstacleDist  = 10.0
            lastSurveyPosition    = .zero
            network.start(ipAddress: serverIP)

            let config = ARWorldTrackingConfiguration()
            config.worldAlignment = .gravity
            if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
                config.sceneReconstruction = .mesh
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

        let cameraTransform = frame.camera.transform
        let col3            = cameraTransform.columns.3
        let currentPos      = SIMD3<Float>(col3.x, col3.y, col3.z)
        let q               = simd_quatf(cameraTransform)

        // Returns distance + which method fired
        let (obstacleDist, obstacleMethod) = detectObstacleDistance(frame: frame)

        // ── Nav Packet (every frame) ──────────────────────────────────
        let navPacket: [String: Any] = [
            "type":            "nav",
            "timestamp":       frame.timestamp,
            "position":        [currentPos.x, currentPos.y, currentPos.z],
            "orientation":     [q.vector.x, q.vector.y, q.vector.z, q.vector.w],
            "obstacle_dist":   obstacleDist,
            "obstacle_method": obstacleMethod   // "RC", "HT", "FP", or "NONE"
        ]
        network.sendPose(navPacket)

        // ── Survey Packet (every 2 metres) ────────────────────────────
        let distMoved = distance(currentPos, lastSurveyPosition)
        if distMoved >= surveyInterval {
            let surveyPacket: [String: Any] = [
                "type":      "survey",
                "timestamp": frame.timestamp,
                "position":  [currentPos.x, currentPos.y, currentPos.z],
                "label":     "Survey Point",
                "note":      "Captured at \(String(format: "%.2f", distMoved))m interval"
            ]
            network.sendPose(surveyPacket)
            lastSurveyPosition = currentPos
            print("📍 Survey packet sent at: \(currentPos)")
        }
    }

    // MARK: - Obstacle Distance Cascade
    // Returns (smoothed distance, method tag)
    // Priority: RC (raycast) → HT (hitTest) → FP (feature points) → NONE
    private func detectObstacleDistance(frame: ARFrame) -> (Float, String) {
        var rawDistance: Float = 10.0
        var method = "NONE"

        if let (d, _) = performRaycastGrid(frame: frame) {
            rawDistance = d
            method      = "RC"
        } else if let d = performHitTestGrid(cameraTransform: frame.camera.transform) {
            rawDistance = d
            method      = "HT"
        } else if let d = featurePointFallback(frame: frame) {
            rawDistance = d
            method      = "FP"
        }

        smoothedObstacleDist = (smoothingAlpha * rawDistance) +
                               ((1.0 - smoothingAlpha) * smoothedObstacleDist)
        return (smoothedObstacleDist, method)
    }

    // MARK: - RC: ARKit session.raycast() 5×5 Grid
    // Uses the reconstructed mesh/planes — more accurate than hitTest but
    // needs a few seconds for ARKit to map the environment first.
    // Must dispatch to main thread to call raycastQuery on ARSCNView.
    // Returns (nearest distance, nearest world position) or nil.
    private func performRaycastGrid(frame: ARFrame) -> (Float, SIMD3<Float>)? {
        var result: (Float, SIMD3<Float>)? = nil
        let semaphore = DispatchSemaphore(value: 0)

        DispatchQueue.main.async { [weak self] in
            defer { semaphore.signal() }
            guard let self = self else { return }

            let view     = self.sceneView
            let bounds   = view.bounds
            let centerX  = bounds.midX
            let centerY  = bounds.midY
            let radius   = self.rayScreenRadius * min(bounds.width, bounds.height)

            let camPos   = SIMD3<Float>(
                frame.camera.transform.columns.3.x,
                frame.camera.transform.columns.3.y,
                frame.camera.transform.columns.3.z
            )

            var bestDist:  Float            = .greatestFiniteMagnitude
            var bestPos:   SIMD3<Float>?    = nil
            var hitCount                    = 0

            let half = (self.rayGridSize - 1) / 2
            for i in 0..<self.rayGridSize {
                for j in 0..<self.rayGridSize {
                    let ox = CGFloat(i - half) / CGFloat(max(1, half))
                    let oy = CGFloat(j - half) / CGFloat(max(1, half))
                    let pt = CGPoint(x: centerX + ox * radius,
                                     y: centerY + oy * radius)

                    // Try existing geometry first, fall back to estimated plane
                    let targets: [ARRaycastQuery.Target] = [
                        .existingPlaneGeometry,
                        .estimatedPlane
                    ]
                    for target in targets {
                        guard let query = view.raycastQuery(
                            from: pt,
                            allowing: target,
                            alignment: .any
                        ) else { continue }

                        let hits = view.session.raycast(query)
                        if let hit = hits.first {
                            let hp = SIMD3<Float>(
                                hit.worldTransform.columns.3.x,
                                hit.worldTransform.columns.3.y,
                                hit.worldTransform.columns.3.z
                            )
                            let d = distance(camPos, hp)
                            if d >= self.minRayDistance && d <= self.maxRayDistance {
                                hitCount += 1
                                if d < bestDist {
                                    bestDist = d
                                    bestPos  = hp
                                }
                            }
                            break  // got a hit for this grid point, move on
                        }
                    }
                }
            }

            if hitCount >= self.requiredHits, let pos = bestPos {
                result = (bestDist, pos)
            }
        }

        semaphore.wait()
        return result
    }

    // MARK: - HT: Legacy hitTest() 5×5 Grid  (unchanged from original)
    // Works on feature points + estimated planes — faster on fresh scenes
    // before raycast has enough map data.
    private func performHitTestGrid(cameraTransform: simd_float4x4) -> Float? {
        let view     = sceneView
        let bounds   = view.bounds
        let centerX  = bounds.midX
        let centerY  = bounds.midY
        let radius   = rayScreenRadius * min(bounds.width, bounds.height)
        let camPos   = SIMD3<Float>(
            cameraTransform.columns.3.x,
            cameraTransform.columns.3.y,
            cameraTransform.columns.3.z
        )

        var nearestDist: Float? = nil
        var hitCount            = 0

        let half = (rayGridSize - 1) / 2
        for i in 0..<rayGridSize {
            for j in 0..<rayGridSize {
                let ox = CGFloat(i - half) / CGFloat(max(1, half))
                let oy = CGFloat(j - half) / CGFloat(max(1, half))
                let pt = CGPoint(x: centerX + ox * radius,
                                  y: centerY + oy * radius)

                let hits = view.hitTest(pt, types: [
                    .featurePoint,
                    .existingPlaneUsingExtent,
                    .estimatedHorizontalPlane
                ])
                if let hit = hits.first {
                    let hp = SIMD3<Float>(
                        hit.worldTransform.columns.3.x,
                        hit.worldTransform.columns.3.y,
                        hit.worldTransform.columns.3.z
                    )
                    let d = distance(camPos, hp)
                    if d >= minRayDistance && d <= maxRayDistance {
                        hitCount += 1
                        nearestDist = nearestDist.map { min($0, d) } ?? d
                    }
                }
            }
        }

        return hitCount >= requiredHits ? nearestDist : nil
    }

    // MARK: - FP: Feature Point Density Cone  (unchanged from original)
    private func featurePointFallback(frame: ARFrame) -> Float? {
        guard let points = frame.rawFeaturePoints?.points else { return nil }

        let worldToCam = frame.camera.transform.inverse
        var count      = 0
        var nearestZ:  Float? = nil

        for p in points {
            let wp    = simd_float4(p.x, p.y, p.z, 1)
            let local = simd_mul(worldToCam, wp)
            let absZ  = -local.z   // positive = in front of camera

            guard absZ > fpNearZ && absZ < fpFarZ else { continue }
            guard abs(local.x) <= fpConeHalfWidth,
                  abs(local.y) <= fpConeHalfHeight else { continue }

            count += 1
            nearestZ = nearestZ.map { min($0, absZ) } ?? absZ
        }

        guard count >= fpDensityThreshold else { return nil }
        return nearestZ.map { max(minRayDistance, min(maxRayDistance, $0)) }
               ?? fpFallbackMinDist
    }
}