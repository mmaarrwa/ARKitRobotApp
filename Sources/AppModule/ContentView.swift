import SwiftUI
import ARKit

struct ContentView: View {
    @StateObject private var arManager = ARManager.shared

    var body: some View {
        ZStack {
            ARViewContainer(arManager: arManager)
                .edgesIgnoringSafeArea(.all)

            VStack {
                Spacer()
                
                // NEW: Input Box for IP
                VStack(spacing: 10) {
                    Text(arManager.statusText)
                        .padding(8)
                        .background(Color.black.opacity(0.5))
                        .foregroundColor(.white)
                        .cornerRadius(8)
                    
                    HStack {
                        // Text Field to type IP
                        TextField("Enter PC IP", text: $arManager.serverIP)
                            .textFieldStyle(RoundedBorderTextFieldStyle())
                            .frame(width: 150)
                            .keyboardType(.numbersAndPunctuation) // Easy typing
                            
                        Button(action: {
                            // Hide keyboard when clicked
                            UIApplication.shared.sendAction(#selector(UIResponder.resignFirstResponder), to: nil, from: nil, for: nil)
                            arManager.toggleStreaming()
                        }) {
                            Text(arManager.isStreaming ? "Stop" : "Start")
                                .padding(8)
                                .background(arManager.isStreaming ? Color.red : Color.green)
                                .foregroundColor(.white)
                                .cornerRadius(8)
                        }
                    }
                }
                .padding()
                .background(Color.black.opacity(0.3)) // Background for visibility
                .cornerRadius(15)
                .padding(.bottom, 20)
            }
        }
        .onAppear {
            arManager.startSessionIfNeeded()
        }
    }
}

struct ARViewContainer: UIViewRepresentable {
    var arManager: ARManager

    func makeUIView(context: Context) -> ARSCNView {
        return arManager.sceneView
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}