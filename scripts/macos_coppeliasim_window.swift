#!/usr/bin/env swift

import AppKit
import CoreGraphics
import Foundation
import ImageIO
import UniformTypeIdentifiers

func fail(_ message: String, status: Int32) -> Never {
  FileHandle.standardError.write(Data((message + "\n").utf8))
  exit(status)
}

if CommandLine.arguments.count == 3 && CommandLine.arguments[1] == "inspect" {
  let inputUrl = URL(fileURLWithPath: CommandLine.arguments[2]) as CFURL
  guard let source = CGImageSourceCreateWithURL(inputUrl, nil) else {
    fail("cannot open GIF for inspection", status: 1)
  }
  let frameCount = CGImageSourceGetCount(source)
  guard frameCount >= 1 else {
    fail("GIF contains no decodable frames", status: 1)
  }
  var width = 0
  var height = 0
  for frameIndex in 0..<frameCount {
    guard let image = CGImageSourceCreateImageAtIndex(source, frameIndex, nil) else {
      fail("cannot decode GIF frame \(frameIndex)", status: 1)
    }
    if frameIndex == 0 {
      width = image.width
      height = image.height
    }
  }
  let result: [String: Any] = [
    "frame_count": frameCount,
    "width": width,
    "height": height,
  ]
  let encoded = try JSONSerialization.data(withJSONObject: result, options: [.sortedKeys])
  FileHandle.standardOutput.write(encoded)
  FileHandle.standardOutput.write(Data("\n".utf8))
  exit(0)
}

if CommandLine.arguments.count >= 5 && CommandLine.arguments[1] == "gif" {
  let outputPath = CommandLine.arguments[2]
  guard let framesPerSecond = Double(CommandLine.arguments[3]), framesPerSecond > 0 else {
    fail("GIF frames per second must be positive", status: 64)
  }
  let framePaths = Array(CommandLine.arguments.dropFirst(4))
  guard framePaths.count >= 2 else {
    fail("GIF requires at least two frames", status: 64)
  }
  let outputUrl = URL(fileURLWithPath: outputPath) as CFURL
  guard let destination = CGImageDestinationCreateWithURL(
    outputUrl, UTType.gif.identifier as CFString, framePaths.count, nil)
  else {
    fail("cannot create GIF destination", status: 1)
  }
  let gifProperties: [CFString: Any] = [
    kCGImagePropertyGIFLoopCount: 0
  ]
  CGImageDestinationSetProperties(destination, [
    kCGImagePropertyGIFDictionary: gifProperties
  ] as CFDictionary)
  let delay = 1.0 / framesPerSecond
  let frameProperties: [CFString: Any] = [
    kCGImagePropertyGIFDictionary: [
      kCGImagePropertyGIFDelayTime: delay,
      kCGImagePropertyGIFUnclampedDelayTime: delay,
    ]
  ]
  let thumbnailOptions: [CFString: Any] = [
    kCGImageSourceCreateThumbnailFromImageAlways: true,
    kCGImageSourceCreateThumbnailWithTransform: true,
    kCGImageSourceThumbnailMaxPixelSize: 1200,
  ]
  var outputWidth = 0
  var outputHeight = 0
  for framePath in framePaths {
    let frameUrl = URL(fileURLWithPath: framePath) as CFURL
    guard let source = CGImageSourceCreateWithURL(frameUrl, nil),
          let image = CGImageSourceCreateThumbnailAtIndex(source, 0, thumbnailOptions as CFDictionary)
    else {
      fail("cannot decode GIF frame: \(framePath)", status: 1)
    }
    if outputWidth == 0 {
      outputWidth = image.width
      outputHeight = image.height
    }
    CGImageDestinationAddImage(destination, image, frameProperties as CFDictionary)
  }
  guard CGImageDestinationFinalize(destination) else {
    fail("failed to finalize GIF", status: 1)
  }
  let result: [String: Any] = [
    "frame_count": framePaths.count,
    "frames_per_second": framesPerSecond,
    "width": outputWidth,
    "height": outputHeight,
  ]
  let encoded = try JSONSerialization.data(withJSONObject: result, options: [.sortedKeys])
  FileHandle.standardOutput.write(encoded)
  FileHandle.standardOutput.write(Data("\n".utf8))
  exit(0)
}

guard CommandLine.arguments.count == 3,
      CommandLine.arguments[1] == "locate",
      let requestedPid = Int32(CommandLine.arguments[2])
else {
  fail(
    "usage: macos_coppeliasim_window locate <pid> | inspect <gif> | gif <output> <fps> <frames...>",
    status: 64)
}

guard CGPreflightScreenCaptureAccess() else {
  fail("Screen Recording permission is unavailable", status: 2)
}

NSRunningApplication(processIdentifier: requestedPid)?.activate(options: [.activateAllWindows])

guard let windowList = CGWindowListCopyWindowInfo(
  [.optionOnScreenOnly, .excludeDesktopElements], kCGNullWindowID) as? [[String: Any]]
else {
  fail("macOS window list is unavailable", status: 3)
}

let candidates = windowList.compactMap { window -> (id: UInt32, width: Int, height: Int, title: String)? in
  guard let ownerPid = window[kCGWindowOwnerPID as String] as? Int32,
        ownerPid == requestedPid,
        let layer = window[kCGWindowLayer as String] as? Int,
        layer == 0,
        let number = window[kCGWindowNumber as String] as? UInt32,
        let boundsDictionary = window[kCGWindowBounds as String] as? NSDictionary,
        let bounds = CGRect(dictionaryRepresentation: boundsDictionary as CFDictionary),
        bounds.width >= 640,
        bounds.height >= 480
  else {
    return nil
  }
  let title = window[kCGWindowName as String] as? String ?? ""
  return (number, Int(bounds.width), Int(bounds.height), title)
}

guard let selected = candidates.max(by: { $0.width * $0.height < $1.width * $1.height }) else {
  fail("no visible CoppeliaSim application window for owned process", status: 3)
}

let payload: [String: Any] = [
  "window_id": selected.id,
  "width": selected.width,
  "height": selected.height,
  "title": selected.title,
  "owner_pid": requestedPid,
]
let encoded = try JSONSerialization.data(withJSONObject: payload, options: [.sortedKeys])
FileHandle.standardOutput.write(encoded)
FileHandle.standardOutput.write(Data("\n".utf8))
