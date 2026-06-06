import Foundation
import ARKit
import SceneKit
import simd

final class ARManager: NSObject, ObservableObject, ARSessionDelegate {
    static let shared = ARManager()

    let sceneView: ARSCNView = {
        let v = ARSCNView(frame: .zero)
        v.autoenablesDefaultLighting = true
        v.debugOptions = [.showFeaturePoints, .showWorldOrigin]
        return v
    }()

    @Published var isStreaming: Bool = false
    @Published var statusText: String = "Idle"        // ← needed by ContentView
    @Published var serverIP: String = "192.168.1.10"  // ← needed by ContentView

    private let network = NetworkManager.shared

    // MARK: - Scan Parameters
    private let numScanColumns = 30
    private let rayScreenRadius: CGFloat = 0.30
    private let maxRayDistance: Float = 3.0
    private let minRayDistance: Float = 0.15
    private let smoothingAlpha: Float = 0.2

    private var smoothedScan: [Float] = Array(repeating: 3.0, count: 30)

    // MARK: - Init
    override init() {
        super.init()
        network.onCommandReceived = { [weak self] command in
            self?.handleRemoteCommand(command)
        }
    }

    // MARK: - Session (needed by ContentView)
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

    // MARK: - toggleStreaming (needed by ContentView Start/Stop button)
    func toggleStreaming() {
        isStreaming.toggle()

        if isStreaming {
            statusText = "Streaming..."
            smoothedScan = Array(repeating: 3.0, count: numScanColumns)

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

        let cameraTransform = frame.camera.transform
        let pos = cameraTransform.columns.3
        let q = simd_quatf(cameraTransform)

        let scanResults = getFullLaserScan(frame: frame)

        for i in 0..<numScanColumns {
            smoothedScan[i] = (smoothingAlpha * scanResults.distances[i]) +
                              ((1.0 - smoothingAlpha) * smoothedScan[i])
        }

        let navPacket: [String: Any] = [
            "type":        "nav",
            "timestamp":   frame.timestamp,
            "position":    [pos.x, pos.y, pos.z],
            "orientation": [q.vector.x, q.vector.y, q.vector.z, q.vector.w],
            "laser_scan":  smoothedScan,
            "confidence":  scanResults.confidence,
            "method":      scanResults.method
        ]
        network.sendPose(navPacket)
    }

    // MARK: - Laser Scan Cascade
    // M1 → M2 → M3 → M4
    private func getFullLaserScan(frame: ARFrame)
        -> (distances: [Float], confidence: Float, method: String) {

        // M1: LiDAR — no main thread needed, reads pixel buffer directly
        if let data = m1_lidar(frame: frame) {
            return (data, 1.0, "M1_LiDAR")
        }

        // M2: Raycast — MUST run on main thread
        var rayData: [Float]? = nil
        let sem2 = DispatchSemaphore(value: 0)
        DispatchQueue.main.async { [weak self] in
            rayData = self?.m2_raycast(frame: frame)
            sem2.signal()
        }
        sem2.wait()
        if let data = rayData { return (data, 0.8, "M2_Raycast") }

        // M3: HitTest — MUST run on main thread
        var hitData: [Float]? = nil
        let sem3 = DispatchSemaphore(value: 0)
        DispatchQueue.main.async { [weak self] in
            hitData = self?.m3_hitTest(frame: frame)
            sem3.signal()
        }
        sem3.wait()
        if let data = hitData { return (data, 0.5, "M3_HitTest") }

        // M4: Cone fallback — always returns something
        return (m4_cone(), 0.2, "M4_Cone")
    }

    // MARK: - M1: LiDAR depth map (30 columns × 5 rows)
    private func m1_lidar(frame: ARFrame) -> [Float]? {
        guard let depthMap = frame.smoothedSceneDepth?.depthMap else { return nil }
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        let w   = CVPixelBufferGetWidth(depthMap)
        let h   = CVPixelBufferGetHeight(depthMap)
        let buf = CVPixelBufferGetBaseAddress(depthMap)!
                    .assumingMemoryBound(to: Float32.self)

        var res = Array(repeating: Float.infinity, count: numScanColumns)
        for col in 0..<numScanColumns {
            let px = Int((Float(col) / Float(numScanColumns)) * Float(w))
            for row in 0..<5 {
                let py = Int((Float(row) / 5.0) * Float(h))
                let d  = buf[py * w + px]
                if d >= minRayDistance && d <= maxRayDistance {
                    res[col] = min(res[col], d)
                }
            }
        }
        // Only return if at least one column got a valid reading
        return res.contains(where: { $0 != .infinity }) ? res : nil
    }

    // MARK: - M2: ARKit session.raycast() — 30 columns
    // Call only from main thread
    private func m2_raycast(frame: ARFrame) -> [Float]? {
        let view = sceneView
        let w    = view.bounds.width
        let midY = view.bounds.midY
        let camPos = simd_float3(frame.camera.transform.columns.3.xyz)

        var res = Array(repeating: Float.infinity, count: numScanColumns)

        for col in 0..<numScanColumns {
            let pt = CGPoint(x: CGFloat(col) / CGFloat(numScanColumns) * w,
                             y: midY)
            // Try existing geometry first, then estimated plane
            for target: ARRaycastQuery.Target in [.existingPlaneGeometry, .estimatedPlane] {
                guard let query = view.raycastQuery(from: pt,
                                                    allowing: target,
                                                    alignment: .any) else { continue }
                if let hit = view.session.raycast(query).first {
                    let hp = simd_float3(hit.worldTransform.columns.3.xyz)
                    let d  = distance(camPos, hp)
                    if d >= minRayDistance && d <= maxRayDistance {
                        res[col] = d
                    }
                    break
                }
            }
        }
        return res.contains(where: { $0 != .infinity }) ? res : nil
    }

    // MARK: - M3: Legacy hitTest() — 30 columns
    // Call only from main thread
    private func m3_hitTest(frame: ARFrame) -> [Float]? {
        let view   = sceneView
        let w      = view.bounds.width
        let midY   = view.bounds.midY
        let camPos = simd_float3(frame.camera.transform.columns.3.xyz)

        var res = Array(repeating: Float.infinity, count: numScanColumns)

        for col in 0..<numScanColumns {
            let pt   = CGPoint(x: CGFloat(col) / CGFloat(numScanColumns) * w,
                                y: midY)
            let hits = view.hitTest(pt, types: [.featurePoint])
            if let hit = hits.first {
                let hp = simd_float3(hit.worldTransform.columns.3.xyz)
                let d  = distance(camPos, hp)
                if d >= minRayDistance && d <= maxRayDistance {
                    res[col] = d
                }
            }
        }
        return res.contains(where: { $0 != .infinity }) ? res : nil
    }

    // MARK: - M4: Cone fallback — returns max distance for all columns
    private func m4_cone() -> [Float] {
        return Array(repeating: maxRayDistance, count: numScanColumns)
    }
}

extension simd_float4 {
    var xyz: simd_float3 { return simd_float3(x, y, z) }
}