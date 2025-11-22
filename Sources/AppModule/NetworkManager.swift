import Foundation
import Network

final class NetworkManager {
    static let shared = NetworkManager()
    private let portNumber: UInt16 = 5005
    private var connection: NWConnection?
    var onCommandReceived: ((String) -> Void)? // Callback for remote commands

    private init() {}

    func start(ipAddress: String) {
        if connection != nil { stop() }
        
        let host = NWEndpoint.Host(ipAddress)
        guard let port = NWEndpoint.Port(rawValue: portNumber) else { return }
        
        // Create UDP connection
        connection = NWConnection(host: host, port: port, using: .udp)
        
        // Start State Update Handler
        connection?.stateUpdateHandler = { state in
            switch state {
            case .ready:
                print("Connected to \(ipAddress)")
                // Once connected, start listening for data from the Laptop
                self.receiveIncomingData()
            case .failed(let error):
                print("Connection failed: \(error)")
            default:
                break
            }
        }
        
        connection?.start(queue: .global())
    }

    func stop() {
        connection?.cancel()
        connection = nil
    }

    func sendPose(_ pose: [String: Any]) {
        guard let data = try? JSONSerialization.data(withJSONObject: pose, options: []) else { return }
        connection?.send(content: data, completion: .contentProcessed({ _ in }))
    }
    

}