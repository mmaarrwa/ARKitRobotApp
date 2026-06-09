import Foundation
import ARKit
import SceneKit
import simd

final class ARManager: NSObject, ObservableObject {
    static let shared = ARManager()

    let sceneView: ARSCNView = {
        let v = ARSCNView(frame: .zero)
        v.autoenablesDefaultLighting = true
        v.debugOptions = [.showFeaturePoints, .showWorldOrigin]
        return v
    }()

    @Published var isStreaming: Bool = false
    @Published var statusText: String = "Idle"
    @Published var serverIP: String = "192.168.1.10"

    private let network = NetworkManager.shared
    private var scanTimer: Timer?

    // --- Configurable parameters ---
    private let numScanColumns = 30
    private let rayScreenRadius: CGFloat = 0.5 // 100% horizontal coverage
    private let numRows = 5
    private let verticalSpread: CGFloat = 0.20  // 40% vertical coverage
    
    // NEW: The "Look Down" Offset (0.5 is center, 0.65 is pointing at the floor)
    private let verticalCenterOffset: CGFloat = 0.65 
    
    private let maxRayDistance: Float = 3.0
    private let minRayDistance: Float = 0.15
    
    // Feature Point Fallback Params
    private let featurePointConeHalfWidth: Float = 0.25
    private let featurePointConeHalfHeight: Float = 0.25
    // NEW: Shift the 3D M4 box downward by 30 centimeters (-0.3m)
    private let featurePointOffsetY: Float = -0.30 
    
    private let featurePointNearZ: Float = -0.2
    private let featurePointFarZ: Float = -2.5
    private let featurePointDensityThreshold = 60

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
        }
        sceneView.session.run(config)
        statusText = "Ready to Connect"
    }

    func handleRemoteCommand(_ command: String) {
        if command == "START" { if !isStreaming { toggleStreaming() } }
        else if command == "STOP" { if isStreaming { toggleStreaming() } }
    }

    func toggleStreaming() {
        isStreaming.toggle()

        if isStreaming {
            statusText = "Streaming..."
            smoothedScan = Array(repeating: 10.0, count: numScanColumns)
            network.start(ipAddress: serverIP)

            let config = ARWorldTrackingConfiguration()
            config.worldAlignment = .gravity
            if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
                config.sceneReconstruction = .mesh
            }
            sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
            
            scanTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
                self?.processAndSendFrame()
            }
        } else {
            statusText = "Stopped"
            network.stop()
            scanTimer?.invalidate()
            scanTimer = nil
        }
    }

    private func processAndSendFrame() {
        guard let frame = sceneView.session.currentFrame else { return }

        let cameraTransform = frame.camera.transform
        let currentPos = cameraTransform.columns.3
        let q = simd_quatf(cameraTransform)

        let (scanArray, confidence, method) = getLaserScan(frame: frame)

        for i in 0..<numScanColumns {
            smoothedScan[i] = (smoothingAlpha * scanArray[i]) + ((1.0 - smoothingAlpha) * smoothedScan[i])
        }

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

    private func getLaserScan(frame: ARFrame) -> (distances: [Float], confidence: Float, method: String) {
        if let data = m1_lidarGrid(frame: frame) { return (data, 1.0, "M1_LiDAR") }
        if let data = m2_raycastGrid(frame: frame) { return (data, 0.8, "M2_Raycast") }
        if let data = m3_hitTestGrid(frame: frame) { return (data, 0.5, "M3_HitTest") }
        return (m4_coneFallback(frame: frame), 0.2, "M4_Cone")
    }

    // MARK: - Sensor Methods
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
            let nx = Float(col) / Float(numScanColumns - 1) * 2.0 - 1.0
            let px = max(0, min(Int((0.5 + (nx * Float(rayScreenRadius))) * Float(w)), w - 1))
            var colMin = Float.infinity
            
            for row in 0..<numRows {
                let ny = Float(row) / Float(max(1, numRows - 1)) * 2.0 - 1.0
                // APPLIED OFFSET HERE
                let normY = Float(verticalCenterOffset) + (ny * Float(verticalSpread))
                let py = max(0, min(Int(normY * Float(h)), h - 1))
                
                let dist = buf[py * w + px]
                if dist >= minRayDistance && dist <= maxRayDistance {
                    colMin = min(colMin, dist)
                    hitCount += 1
                }
            }
            scan[col] = colMin
        }
        return hitCount > 0 ? scan : nil
    }

    private func m2_raycastGrid(frame: ARFrame) -> [Float]? {
        let view = self.sceneView
        let bounds = view.bounds
        if bounds.width == 0 { return nil }

        // APPLIED OFFSET HERE
        let centerY = bounds.height * verticalCenterOffset 
        let camPos = SIMD3<Float>(frame.camera.transform.columns.3.x, frame.camera.transform.columns.3.y, frame.camera.transform.columns.3.z)
        
        var scan = Array(repeating: Float.infinity, count: numScanColumns)
        var hitCount = 0
        
        for col in 0..<numScanColumns {
            let nx = CGFloat(col) / CGFloat(numScanColumns - 1) * 2.0 - 1.0
            var colMin = Float.infinity
            for row in 0..<numRows {
                let ny = CGFloat(row) / CGFloat(max(1, numRows - 1)) * 2.0 - 1.0
                // Using centerY instead of bounds.midY
                let samplePoint = CGPoint(x: bounds.midX + nx * (rayScreenRadius * bounds.width), 
                                          y: centerY + ny * (verticalSpread * bounds.height))

                guard let query = view.raycastQuery(from: samplePoint, allowing: .estimatedPlane, alignment: .any) else { continue }
                if let hit = view.session.raycast(query).first {
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
        return hitCount > 0 ? scan : nil
    }

    private func m3_hitTestGrid(frame: ARFrame) -> [Float]? {
        let view = self.sceneView
        let bounds = view.bounds
        if bounds.width == 0 { return nil }
        
        // APPLIED OFFSET HERE
        let centerY = bounds.height * verticalCenterOffset 
        let camPos = SIMD3<Float>(frame.camera.transform.columns.3.x, frame.camera.transform.columns.3.y, frame.camera.transform.columns.3.z)
        
        var scan = Array(repeating: Float.infinity, count: numScanColumns)
        var hitCount = 0
        
        for col in 0..<numScanColumns {
            let nx = CGFloat(col) / CGFloat(numScanColumns - 1) * 2.0 - 1.0
            var colMin = Float.infinity
            for row in 0..<numRows {
                let ny = CGFloat(row) / CGFloat(max(1, numRows - 1)) * 2.0 - 1.0
                // Using centerY instead of bounds.midY
                let samplePoint = CGPoint(x: bounds.midX + nx * (rayScreenRadius * bounds.width), 
                                          y: centerY + ny * (verticalSpread * bounds.height))

                if let hit = view.hitTest(samplePoint, types: [.featurePoint, .estimatedHorizontalPlane]).first {
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
        return hitCount > 0 ? scan : nil
    }

    private func m4_coneFallback(frame: ARFrame) -> [Float] {
        guard let points = frame.rawFeaturePoints?.points else { return Array(repeating: 10.0, count: numScanColumns) }

        let worldToCamera = frame.camera.transform.inverse
        var count = 0
        var nearestZ: Float? = nil

        for p in points {
            let local = simd_mul(worldToCamera, simd_float4(p.x, p.y, p.z, 1))
            if local.z < featurePointNearZ && local.z > featurePointFarZ {
                // APPLIED 3D OFFSET HERE (local.y - featurePointOffsetY)
                if abs(local.x) <= featurePointConeHalfWidth && abs(local.y - featurePointOffsetY) <= featurePointConeHalfHeight {
                    count += 1
                    if nearestZ == nil || abs(local.z) < nearestZ! { nearestZ = abs(local.z) }
                }
            }
        }

        let finalDist = (count >= featurePointDensityThreshold && nearestZ != nil) ? max(minRayDistance, min(maxRayDistance, nearestZ!)) : 10.0
        return Array(repeating: finalDist, count: numScanColumns)
    }
}