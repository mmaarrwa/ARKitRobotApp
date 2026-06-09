import Foundation
import ARKit
import SceneKit
import simd

final class ARManager: NSObject, ObservableObject, ARSessionDelegate {
    static let shared = ARManager()

    let sceneView: ARSCNView = {
        let v = ARSCNView(frame: .zero)
        v.autoenablesDefaultLighting = true
        v.debugOptions = [.showFeaturePoints, .showWorldOrigin] // Kept the visuals!
        return v
    }()

    @Published var isStreaming: Bool = false
    @Published var statusText: String = "Idle"
    @Published var serverIP: String = "192.168.1.10"

    private let network = NetworkManager.shared

    // --- Configurable parameters ---
    private let numScanColumns = 30           // The 1x30 LaserScan for ROS
    private let rayScreenRadius: CGFloat = 0.30 // Spans 30% of the screen center
    private let numRows = 5                   // 5 vertical rows to flatten
    private let verticalSpread: CGFloat = 0.10  // 10% vertical spread
    
    private let maxRayDistance: Float = 3.0
    private let minRayDistance: Float = 0.15
    
    // Feature Point Fallback Params
    private let featurePointConeHalfWidth: Float = 0.25
    private let featurePointConeHalfHeight: Float = 0.25
    private let featurePointNearZ: Float = -0.2
    private let featurePointFarZ: Float = -2.5
    private let featurePointDensityThreshold = 60
    private let densityFallbackMinDistance: Float = 0.4

    // --- Smoothing ---
    private let smoothingAlpha: Float = 0.2
    private var smoothedScan: [Float] = Array(repeating: 10.0, count: 30)

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
            smoothedScan = Array(repeating: 10.0, count: numScanColumns) // Reset
            
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

        // 1. Get Current Pose & Orientation
        let cameraTransform = frame.camera.transform
        let col3 = cameraTransform.columns.3
        let currentPos = SIMD3<Float>(col3.x, col3.y, col3.z)
        let q = simd_quatf(cameraTransform)

        // 2. Cascade Logic (Get the 30-array, confidence, and method)
        let (scanArray, confidence, method) = getLaserScan(frame: frame)

        // 3. Smooth the Array
        for i in 0..<numScanColumns {
            smoothedScan[i] = (smoothingAlpha * scanArray[i]) + ((1.0 - smoothingAlpha) * smoothedScan[i])
        }

        // 4. Send the Navigation Packet
        let navPacket: [String: Any] = [
            "type": "nav",
            "timestamp": frame.timestamp,
            "position": [currentPos.x, currentPos.y, currentPos.z],
            "orientation": [q.vector.x, q.vector.y, q.vector.z, q.vector.w],
            "laser_scan": smoothedScan,
            "confidence": confidence,
            "method": method
        ]
        network.sendPose(navPacket)
    }

    // MARK: - Obstacle Cascade Logic

    private func getLaserScan(frame: ARFrame) -> (distances: [Float], confidence: Float, method: String) {
        if let data = m1_lidarGrid(frame: frame) { return (data, 1.0, "M1_LiDAR") }
        if let data = m2_raycastGrid(frame: frame) { return (data, 0.8, "M2_Raycast") }
        if let data = m3_hitTestGrid(frame: frame) { return (data, 0.5, "M3_HitTest") }
        
        return (m4_coneFallback(frame: frame), 0.2, "M4_Cone")
    }

    // MARK: - Individual Methods (Flattening 5 rows into 30 cols)

    private func m1_lidarGrid(frame: ARFrame) -> [Float]? {
        guard let depthData = frame.smoothedSceneDepth ?? frame.sceneDepth else { return nil }
        let depthMap = depthData.depthMap
        
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        let w = CVPixelBufferGetWidth(depthMap)
        let h = CVPixelBufferGetHeight(depthMap)
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return nil }
        let buf = baseAddress.assumingMemoryBound(to: Float32.self)
        
        var scan = Array(repeating: Float.infinity, count: numScanColumns)
        var hitCount = 0
        
        for col in 0..<numScanColumns {
            let nx = Float(col) / Float(numScanColumns - 1) * 2.0 - 1.0 // -1 to +1
            let normX = 0.5 + (nx * Float(rayScreenRadius))
            let px = max(0, min(Int(normX * Float(w)), w - 1))
            
            var colMin = Float.infinity
            for row in 0..<numRows {
                let ny = Float(row) / Float(max(1, numRows - 1)) * 2.0 - 1.0
                let normY = 0.5 + (ny * Float(verticalSpread))
                let py = max(0, min(Int(normY * Float(h)), h - 1))
                
                let dist = buf[py * w + px]
                if dist >= minRayDistance && dist <= maxRayDistance {
                    colMin = min(colMin, dist)
                    hitCount += 1
                }
            }
            scan[col] = colMin
        }
        
        if hitCount > 0 { return scan }
        return nil
    }

    private func m2_raycastGrid(frame: ARFrame) -> [Float]? {
        let view = self.sceneView
        let bounds = view.bounds
        if bounds.width == 0 { return nil }

        let center = CGPoint(x: bounds.midX, y: bounds.midY)
        let camPos = SIMD3<Float>(frame.camera.transform.columns.3.x, 
                                  frame.camera.transform.columns.3.y, 
                                  frame.camera.transform.columns.3.z)
        
        var scan = Array(repeating: Float.infinity, count: numScanColumns)
        var hitCount = 0
        
        for col in 0..<numScanColumns {
            let nx = CGFloat(col) / CGFloat(numScanColumns - 1) * 2.0 - 1.0
            var colMin = Float.infinity
            
            for row in 0..<numRows {
                let ny = CGFloat(row) / CGFloat(max(1, numRows - 1)) * 2.0 - 1.0
                let samplePoint = CGPoint(x: center.x + nx * (rayScreenRadius * bounds.width),
                                          y: center.y + ny * (verticalSpread * bounds.height))

                guard let query = view.raycastQuery(from: samplePoint, allowing: .estimatedPlane, alignment: .any) else { continue }
                let results = view.session.raycast(query)
                
                if let hit = results.first {
                    let hitPos = SIMD3<Float>(hit.worldTransform.columns.3.x, hit.worldTransform.columns.3.y, hit.worldTransform.columns.3.z)
                    let d = distance(camPos, hitPos)
                    if d >= minRayDistance && d <= maxRayDistance {
                        colMin = min(colMin, d)
                        hitCount += 1
                    }
                }
            }
            scan[col] = colMin
        }
        if hitCount > 0 { return scan }
        return nil
    }

    private func m3_hitTestGrid(frame: ARFrame) -> [Float]? {
        let view = self.sceneView
        let bounds = view.bounds
        if bounds.width == 0 { return nil }
        
        let center = CGPoint(x: bounds.midX, y: bounds.midY)
        let camPos = SIMD3<Float>(frame.camera.transform.columns.3.x, 
                                  frame.camera.transform.columns.3.y, 
                                  frame.camera.transform.columns.3.z)
        
        var scan = Array(repeating: Float.infinity, count: numScanColumns)
        var hitCount = 0
        
        for col in 0..<numScanColumns {
            let nx = CGFloat(col) / CGFloat(numScanColumns - 1) * 2.0 - 1.0
            var colMin = Float.infinity
            
            for row in 0..<numRows {
                let ny = CGFloat(row) / CGFloat(max(1, numRows - 1)) * 2.0 - 1.0
                let samplePoint = CGPoint(x: center.x + nx * (rayScreenRadius * bounds.width),
                                          y: center.y + ny * (verticalSpread * bounds.height))

                let results = view.hitTest(samplePoint, types: [.featurePoint, .estimatedHorizontalPlane])
                if let hit = results.first {
                    let hitPos = SIMD3<Float>(hit.worldTransform.columns.3.x, hit.worldTransform.columns.3.y, hit.worldTransform.columns.3.z)
                    let d = distance(camPos, hitPos)
                    if d >= minRayDistance && d <= maxRayDistance {
                        colMin = min(colMin, d)
                        hitCount += 1
                    }
                }
            }
            scan[col] = colMin
        }
        if hitCount > 0 { return scan }
        return nil
    }

    private func m4_coneFallback(frame: ARFrame) -> [Float] {
        guard let points = frame.rawFeaturePoints?.points else { 
            return Array(repeating: 10.0, count: numScanColumns) 
        }

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

        let finalDist: Float
        if count >= featurePointDensityThreshold, let nz = nearestZ {
            finalDist = max(minRayDistance, min(maxRayDistance, nz))
        } else {
            finalDist = 10.0 // Path clear
        }
        
        return Array(repeating: finalDist, count: numScanColumns)
    }
}