#!/usr/bin/env python3

import argparse
import json
import os
from pathlib import Path
import shutil

from saleae import automation

def resolve_logic2_binary(explicit_path: str | None) -> Path:
    if explicit_path is not None:
        path = Path(explicit_path).expanduser().resolve()
        if path.exists():
            return path
        raise FileNotFoundError(f"Logic 2 binary not found: {path}")

    resolved = shutil.which("saleae-logic2")
    if resolved is None:
        raise FileNotFoundError("Could not locate `saleae-logic2` on PATH. Pass --logic2.")

    return Path(resolved).resolve()


def configure_custom_analyzers_path(analyzers_dir: Path) -> Path:
    config_path = Path.home() / ".config" / "Logic" / "config.json"
    config = json.loads(config_path.read_text())
    config["customAnalyzerPaths"] = [str(analyzers_dir)]
    config_path.write_text(json.dumps(config, indent=4) + "\n")
    return config_path


def main() -> int:
    root = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser(description="Validate the FlexRay analyzer against a Logic 2 capture.")
    parser.add_argument("--logic2", help="Path to the Logic 2 binary or AppImage.")
    parser.add_argument("--capture", default=str(root / "assets" / "SP2018_FlexRay.sal"), help="Path to the .sal capture.")
    parser.add_argument("--output-dir", default=str(root / "build" / "automation"), help="Directory for exported CSV files.")
    parser.add_argument("--analyzers-dir", default=str(root / "build" / "Analyzers"), help="Directory containing libFlexRayAnalyzer.so.")
    parser.add_argument("--channel", type=int, default=0, help="Digital channel index for the FlexRay signal.")
    parser.add_argument("--bitrate", type=int, default=10_000_000, choices=[2_500_000, 5_000_000, 10_000_000], help="FlexRay bit rate.")
    parser.add_argument("--crc-channel", default="A", choices=["A", "B", "a", "b"], help="FlexRay CRC channel.")
    parser.add_argument("--sample-point", type=int, default=62, help="Sampling point percentage.")
    parser.add_argument("--invert", action="store_true", help="Invert the digital input before decoding.")
    parser.add_argument("--port", type=int, default=10430, help="Automation API port.")
    parser.add_argument("--timeout", type=float, default=30.0, help="Seconds to wait for Logic 2 to launch.")
    args = parser.parse_args()

    logic2_path = resolve_logic2_binary(args.logic2)
    capture_path = Path(args.capture).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    analyzers_dir = Path(args.analyzers_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if analyzers_dir.exists() is False:
        raise FileNotFoundError(f"Analyzer output directory not found: {analyzers_dir}")

    os.environ.setdefault("APPIMAGE_EXTRACT_AND_RUN", "1")
    config_path = configure_custom_analyzers_path(analyzers_dir)

    manager = automation.Manager.launch(
        application_path=str(logic2_path),
        connect_timeout_seconds=args.timeout,
        port=args.port,
    )

    try:
        capture = manager.load_capture(str(capture_path))
        try:
            analyzer = capture.add_analyzer(
                "FlexRay",
                settings={
                    "Input Channel": args.channel,
                    "Bit Rate (Bits/s)": args.bitrate,
                    "CRC Channel": 0 if args.crc_channel.upper() == "A" else 1,
                    "Sample Point (%)": args.sample_point,
                    "Invert Input": args.invert,
                },
            )

            legacy_csv = output_dir / "flexray_export.csv"
            table_csv = output_dir / "flexray_table.csv"
            raw_dir = output_dir / "raw"
            raw_dir.mkdir(parents=True, exist_ok=True)

            capture.legacy_export_analyzer(str(legacy_csv), analyzer, automation.RadixType.HEXADECIMAL)
            capture.export_data_table(
                str(table_csv),
                [automation.DataTableExportConfiguration(analyzer=analyzer, radix=automation.RadixType.HEXADECIMAL)],
            )
            capture.export_raw_data_csv(str(raw_dir), digital_channels=[args.channel])

            print(f"Logic 2 binary: {logic2_path}")
            print(f"Config file:    {config_path}")
            print(f"Analyzers dir:  {analyzers_dir}")
            print(f"Capture:       {capture_path}")
            print(f"Legacy export: {legacy_csv}")
            print(f"Data table:    {table_csv}")
            print(f"Raw export:    {raw_dir / 'digital.csv'}")
        finally:
            capture.close()
    finally:
        manager.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
