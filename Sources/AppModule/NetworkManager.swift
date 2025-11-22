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
        connection?.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready:
                print("Connected to \(ipAddress)")
                // Once connected, start listening for data from the Laptop
                self?.receiveIncomingData()
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
    
    // MARK: - Missing Function Added
    // This listens for "START" or "STOP" from the laptop
    private func receiveIncomingData() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] (data, _, isComplete, error) in
            guard let self = self else { return }
            
            if let error = error {
                print("Receive error: \(error)")
                return
            }
            
            if let data = data, !data.isEmpty, let message = String(data: data, encoding: .utf8) {
                print("Received command: \(message)")
                DispatchQueue.main.async {
                    // Clean up the string (remove newlines) and notify ARManager
                    self.onCommandReceived?(message.trimmingCharacters(in: .whitespacesAndNewlines))
                }
            }
            
            if isComplete {
                print("Connection closed by remote")
            } else {
                // Keep listening for the next command
                self.receiveIncomingData()
            }
        }
    }
}