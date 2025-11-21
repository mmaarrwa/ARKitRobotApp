// swift-tools-version: 5.9
import PackageDescription

// We removed 'import AppleProductTypes' which caused the error

let package = Package(
    name: "ARKitPoseStreamer",
    platforms: [.iOS("16.0")],
    products: [
        // We changed '.iOSApplication' to '.executable'
        // This is the standard way to define an app in SPM
        .executable(
            name: "ARKitPoseStreamer",
            targets: ["AppModule"]
        )
    ],
    targets: [
        .executableTarget(
            name: "AppModule",
            path: "Sources",
            // This is new: We must manually link the Info.plist file
            // so the build system can find the camera permission, etc.
            resources: [
                .process("../Info.plist")
            ]
        )
    ]
)