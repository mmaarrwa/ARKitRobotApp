import Foundation
import ARKit
import SceneKit
import simd

final class ARManager: NSObject, ObservableObject, ARSessionDelegate {
    static let shared = ARManager()

    let sceneView: ARSCNView = {
        let v = ARSCNView(frame: .zero)
        v.autoenablesDefaultLighting = true
        // Visualizing rays and feature points for debugging
        v.debugOptions = [.showFeaturePoints, .showWorldOrigin]
        return v
    }()

    @Published var isStreaming: Bool = false
    @Published var statusText: String = "Idle"
    @Published var serverIP: String = "192.168.1.10"

    private let network = NetworkManager.shared
    
    // --- Scanning Parameters ---
    private let numScanColumns = 30
    private let rayScreenRadius: CGFloat = 0.30 
    private let maxRayDistance: Float = 3.0
    private let minRayDistance: Float = 0.15
    private let smoothingAlpha: Float = 0.2
    
    private var smoothedScan: [Float] = Array(repeating: 3.0, count: 30)

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
        }
        sceneView.session.run(config)
        sceneView.session.delegate = self
    }

    func toggleStreaming() {
        isStreaming.toggle()
        if isStreaming { network.start(ipAddress: serverIP) } else { network.stop() }
    }

    func handleRemoteCommand(_ command: String) {
        if command == "START" { if !isStreaming { toggleStreaming() } }
        else if command == "STOP" { if isStreaming { toggleStreaming() } }
    }

    // MARK: - ARSessionDelegate
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard isStreaming else { return }

        let cameraTransform = frame.camera.transform
        let pos = cameraTransform.columns.3
        let q = simd_quatf(cameraTransform)

        let scanResults = getFullLaserScan(frame: frame)
        
        // Smoothing the array
        for i in 0..<numScanColumns {
            smoothedScan[i] = (smoothingAlpha * scanResults.distances[i]) + ((1.0 - smoothingAlpha) * smoothedScan[i])
        }

        let navPacket: [String: Any] = [
            "type": "nav",
            "timestamp": frame.timestamp,
            "position": [pos.x, pos.y, pos.z],
            "orientation": [q.vector.x, q.vector.y, q.vector.z, q.vector.w],
            "laser_scan": smoothedScan,
            "confidence": scanResults.confidence,
            "method": scanResults.method
        ]
        network.sendPose(navPacket)
    }

    private func getFullLaserScan(frame: ARFrame) -> (distances: [Float], confidence: Float, method: String) {
        // [M1] LiDAR
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh),
           let data = m1_lidar(frame: frame) { return (data, 1.0, "M1_LiDAR") }
        
        // [M2] Raycast
        if let data = m2_raycast(frame: frame) { return (data, 0.8, "M2_Raycast") }
        
        // [M3] HitTest
        if let data = m3_hitTest(frame: frame) { return (data, 0.5, "M3_HitTest") }
        
        // [M4] Fallback
        return (m4_cone(frame: frame), 0.2, "M4_Cone")
    }

    // MARK: - Method Implementations
    private func m1_lidar(frame: ARFrame) -> [Float]? {
        guard let depthMap = frame.smoothedSceneDepth?.depthMap else { return nil }
        CVPixelBufferLockBaseAddress(depthMap, .readOnly); defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        let w = CVPixelBufferGetWidth(depthMap), h = CVPixelBufferGetHeight(depthMap)
        let buf = CVPixelBufferGetBaseAddress(depthMap)!.assumingMemoryBound(to: Float32.self)
        
        var res = Array(repeating: Float.infinity, count: numScanColumns)
        for col in 0..<numScanColumns {
            let px = Int((CGFloat(col)/CGFloat(numScanColumns)) * CGFloat(w))
            for row in 0..<5 {
                let py = Int((CGFloat(row)/5.0) * CGFloat(h))
                let d = buf[py * w + px]
                if d >= minRayDistance && d <= maxRayDistance { res[col] = min(res[col], d) }
            }
        }
        return res
    }

    private func m2_raycast(frame: ARFrame) -> [Float]? {
        var res = Array(repeating: Float.infinity, count: numScanColumns)
        let view = sceneView
        for col in 0..<numScanColumns {
            let samplePoint = CGPoint(x: CGFloat(col)/CGFloat(numScanColumns) * view.bounds.width, y: view.bounds.midY)
            if let query = view.raycastQuery(from: samplePoint, allowing: .estimatedPlane, alignment: .any),
               let hit = view.session.raycast(query).first {
                res[col] = distance(simd_float3(frame.camera.transform.columns.3.xyz), simd_float3(hit.worldTransform.columns.3.xyz))
            }
        }
        return res.contains(where: { $0 != .infinity }) ? res : nil
    }

    private func m3_hitTest(frame: ARFrame) -> [Float]? {
        var res = Array(repeating: Float.infinity, count: numScanColumns)
        let view = sceneView
        for col in 0..<numScanColumns {
            let samplePoint = CGPoint(x: CGFloat(col)/CGFloat(numScanColumns) * view.bounds.width, y: view.bounds.midY)
            let hits = view.hitTest(samplePoint, types: [.featurePoint])
            if let hit = hits.first {
                res[col] = distance(simd_float3(frame.camera.transform.columns.3.xyz), simd_float3(hit.worldTransform.columns.3.xyz))
            }
        }
        return res.contains(where: { $0 != .infinity }) ? res : nil
    }

    private func m4_cone() -> [Float] {
        return Array(repeating: 3.0, count: numScanColumns) // Simplified fallback
    }
}

// Extension to help with Matrix math
extension simd_float4 { var xyz: simd_float3 { return simd_float3(x, y, z) } }