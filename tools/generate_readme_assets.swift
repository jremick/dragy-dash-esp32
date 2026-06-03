import AppKit
import Foundation

struct Panel {
  let title: String
  let speed: String
  let rows: [(String, String)]
}

let panels = [
  Panel(title: "Page 1", speed: "88", rows: [("AVG 5 KM", "86 km/h"), ("SATS", "18"), ("ACCURACY", "1.2 m")]),
  Panel(title: "Page 2", speed: "88", rows: [("AVG 1 KM", "84"), ("AVG 5 KM", "86"), ("AVG 10 KM", "82")]),
  Panel(title: "Page 3", speed: "88", rows: [("PEAK", "112 km/h"), ("DIST", "6.42 km"), ("ALT", "49 m")])
]

let scale = 2.0
let panelSize = CGSize(width: 320, height: 170)
let gap: CGFloat = 20
let margin: CGFloat = 24
let titleHeight: CGFloat = 30
let imageSize = CGSize(
  width: margin * 2 + panelSize.width * CGFloat(panels.count) + gap * CGFloat(panels.count - 1),
  height: margin * 2 + titleHeight + panelSize.height
)

let bitmap = NSBitmapImageRep(
  bitmapDataPlanes: nil,
  pixelsWide: Int(imageSize.width * scale),
  pixelsHigh: Int(imageSize.height * scale),
  bitsPerSample: 8,
  samplesPerPixel: 4,
  hasAlpha: true,
  isPlanar: false,
  colorSpaceName: .deviceRGB,
  bytesPerRow: 0,
  bitsPerPixel: 0
)!

NSGraphicsContext.saveGraphicsState()
NSGraphicsContext.current = NSGraphicsContext(bitmapImageRep: bitmap)
NSGraphicsContext.current?.cgContext.scaleBy(x: scale, y: scale)

func drawText(_ text: String, in rect: CGRect, font: NSFont, color: NSColor, alignment: NSTextAlignment = .left) {
  let paragraph = NSMutableParagraphStyle()
  paragraph.alignment = alignment
  paragraph.lineBreakMode = .byTruncatingTail
  let attributes: [NSAttributedString.Key: Any] = [
    .font: font,
    .foregroundColor: color,
    .paragraphStyle: paragraph
  ]
  text.draw(in: rect, withAttributes: attributes)
}

func roundedRect(_ rect: CGRect, radius: CGFloat, color: NSColor) {
  color.setFill()
  NSBezierPath(roundedRect: rect, xRadius: radius, yRadius: radius).fill()
}

roundedRect(CGRect(origin: .zero, size: imageSize), radius: 0, color: NSColor(calibratedWhite: 0.94, alpha: 1))

for (index, panel) in panels.enumerated() {
  let x = margin + CGFloat(index) * (panelSize.width + gap)
  let titleRect = CGRect(x: x, y: imageSize.height - margin - 22, width: panelSize.width, height: 20)
  drawText(panel.title, in: titleRect, font: .systemFont(ofSize: 15, weight: .semibold), color: NSColor(calibratedWhite: 0.28, alpha: 1), alignment: .center)

  let frame = CGRect(x: x, y: margin, width: panelSize.width, height: panelSize.height)
  roundedRect(frame, radius: 18, color: .black)

  let screen = frame.insetBy(dx: 10, dy: 10)
  roundedRect(screen, radius: 10, color: NSColor(calibratedWhite: 0.025, alpha: 1))

  func tftRect(x: CGFloat, y: CGFloat, width: CGFloat, height: CGFloat) -> CGRect {
    CGRect(
      x: screen.minX + x,
      y: screen.maxY - y - height,
      width: width,
      height: height
    )
  }

  drawText(panel.speed, in: tftRect(x: 8, y: 36, width: 150, height: 78), font: .monospacedDigitSystemFont(ofSize: 72, weight: .semibold), color: .white, alignment: .center)
  drawText("km/h", in: tftRect(x: 8, y: 106, width: 150, height: 22), font: .systemFont(ofSize: 18, weight: .semibold), color: NSColor(calibratedWhite: 0.42, alpha: 1), alignment: .center)

  for (rowIndex, row) in panel.rows.enumerated() {
    let y = CGFloat(4 + rowIndex * 52)
    drawText(row.0, in: tftRect(x: 166, y: y, width: 120, height: 18), font: .systemFont(ofSize: 11, weight: .bold), color: NSColor(calibratedWhite: 0.42, alpha: 1), alignment: .right)
    drawText(row.1, in: tftRect(x: 148, y: y + 18, width: 138, height: 26), font: .monospacedDigitSystemFont(ofSize: 21, weight: .semibold), color: .white, alignment: .right)
  }

  drawText("READY  84%  3D", in: tftRect(x: 6, y: 132, width: 210, height: 18), font: .systemFont(ofSize: 12, weight: .semibold), color: NSColor(calibratedWhite: 0.42, alpha: 1))
  NSColor.systemGreen.setFill()
  NSBezierPath(ovalIn: tftRect(x: 290, y: 139, width: 10, height: 10)).fill()
}

NSGraphicsContext.restoreGraphicsState()

let outputURL = URL(fileURLWithPath: "docs/assets/esp32-pages.png")
try FileManager.default.createDirectory(at: outputURL.deletingLastPathComponent(), withIntermediateDirectories: true)
guard let data = bitmap.representation(using: .png, properties: [:]) else {
  fatalError("Failed to encode PNG")
}
try data.write(to: outputURL)
print(outputURL.path)
