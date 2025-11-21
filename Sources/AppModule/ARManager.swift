import Foundation
import ARKit
import SceneKit
import simd

final class ARManager: NSObject, ObservableObject, ARSessionDelegate {
    static let shared = ARManager()

    let sceneView: ARSCNView = {
        let v = ARSCNView(frame: .zero)
        v.autoenablesDefaultLighting = true
        // ✅ NEW: Show yellow feature points for debugging
        v.debugOptions = [ARSCNDebugOptions.showFeaturePoints] 
        return v
    }()

    @Published var isStreaming: Bool = false
    @Published var statusText: String = "Idle"
    @Published var serverIP: String = "192.168.1.10" // Default IP

    private let network = NetworkManager.shared

    override init() {
        super.init()
        // Handle commands coming from NetworkManager
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

    // ✅ NEW: Handle commands from Laptop
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
            
            // ✅ FIX: Reset Origin to (0,0,0) NOW
            let config = ARWorldTrackingConfiguration()
            config.worldAlignment = .gravity
            // .resetTracking makes the current phone position the new (0,0,0)
            sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
            
        } else {
            statusText = "Stopped"
            network.stop()
        }
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard isStreaming else { return }

        // 1. Get Pose
        let t = frame.camera.transform
        let col3 = t.columns.3
        let pos = SIMD3<Float>(col3.x, col3.y, col3.z)
        
        // Orientation
        let q = simd_quatf(t)

        // 2. ✅ NEW: Obstacle Detection (Feature Points)
        var minDistance: Float = 10.0 // Start with a far distance (meters)
        
        if let points = frame.rawFeaturePoints?.points {
            // Loop through points to find the closest one to the camera
            for point in points {
                // Distance between camera (pos) and feature point (point)
                let dist = distance(pos, point)
                if dist < minDistance {
                    minDistance = dist
                }
            }
        }

        // 3. Build Data Payload
        let pose: [String: Any] = [
            "timestamp": frame.timestamp,
            "position": [pos.x, pos.y, pos.z],
            "orientation": [q.vector.x, q.vector.y, q.vector.z, q.vector.w],
            "obstacle_dist": minDistance // Send distance to nearest object
        ]

        network.sendPose(pose)
    }
}