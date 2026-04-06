#!/usr/bin/env python3

import argparse
import csv
import json
import os
from collections import Counter
from pathlib import Path
import shutil
import sys
import time
from zipfile import ZIP_DEFLATED, ZipFile

from saleae import automation

def resolve_logic2_executable(path: Path) -> Path:
    if path.suffix == ".app":
        path = path / "Contents" / "MacOS" / "Logic"

    return path.expanduser().resolve()


def resolve_logic2_binary(explicit_path: str | None) -> Path:
    if explicit_path is not None:
        path = resolve_logic2_executable(Path(explicit_path))
        if path.exists():
            return path
        raise FileNotFoundError(f"Logic 2 binary not found: {path}")

    if sys.platform == "darwin":
        for candidate in (
            Path("/Applications/Saleae Logic.app"),
            Path.home() / "Applications" / "Saleae Logic.app",
        ):
            resolved = resolve_logic2_executable(candidate)
            if resolved.exists():
                return resolved

    resolved = shutil.which("saleae-logic2")
    if resolved is None:
        raise FileNotFoundError("Could not locate Logic 2. Pass --logic2 or install `saleae-logic2` on PATH.")

    return Path(resolved).resolve()


def resolve_logic2_config_path() -> Path:
    if sys.platform == "darwin":
        return Path.home() / "Library" / "Application Support" / "Logic" / "config.json"

    return Path.home() / ".config" / "Logic" / "config.json"


def configure_custom_analyzers_path(analyzers_dir: Path) -> Path:
    config_path = resolve_logic2_config_path()
    config_path.parent.mkdir(parents=True, exist_ok=True)

    if config_path.exists():
        config = json.loads(config_path.read_text())
    else:
        config = {}

    config["customAnalyzerPaths"] = [str(analyzers_dir)]
    config_path.write_text(json.dumps(config, indent=4) + "\n")
    return config_path


def add_flexray_analyzer(capture: automation.Capture, args: argparse.Namespace) -> automation.AnalyzerHandle:
    return capture.add_analyzer(
        "FlexRay",
        settings={
            "Input Channel": args.channel,
            "Bit Rate (Bits/s)": args.bitrate,
            "CRC Channel": 0 if args.crc_channel.upper() == "A" else 1,
            "Sample Point (%)": args.sample_point,
            "Invert Input": args.invert,
        },
    )


def close_capture_if_present(capture: automation.Capture | None) -> None:
    if capture is None:
        return

    try:
        capture.close()
    except automation.errors.InvalidRequestError as exc:
        if 'does not exist' not in str(exc):
            raise


def create_capture_without_saved_analyzers(source_path: Path, output_path: Path) -> Path:
    with ZipFile(source_path, 'r') as source_zip:
        meta = json.loads(source_zip.read('meta.json'))
        meta['data']['analyzers'] = []
        meta['data']['highLevelAnalyzers'] = []

        with ZipFile(output_path, 'w', compression=ZIP_DEFLATED) as output_zip:
            for info in source_zip.infolist():
                if info.filename == 'meta.json':
                    continue

                output_zip.writestr(info.filename, source_zip.read(info.filename))

            output_zip.writestr('meta.json', json.dumps(meta, separators=(',', ':')).encode('utf-8'))

    return output_path


def export_capture_data(
    capture: automation.Capture, analyzer: automation.AnalyzerHandle, output_dir: Path, channel: int
) -> tuple[Path, Path, Path]:
    legacy_csv = output_dir / "flexray_export.csv"
    table_csv = output_dir / "flexray_table.csv"
    raw_dir = output_dir / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)

    capture.legacy_export_analyzer(str(legacy_csv), analyzer, automation.RadixType.HEXADECIMAL)
    capture.export_data_table(
        str(table_csv),
        [automation.DataTableExportConfiguration(analyzer=analyzer, radix=automation.RadixType.HEXADECIMAL)],
    )
    capture.export_raw_data_csv(str(raw_dir), digital_channels=[channel])
    return legacy_csv, table_csv, raw_dir


def summarize_legacy_export(export_path: Path) -> dict[str, object]:
    frame_ids: Counter[str] = Counter()
    summary = {
        "rows": 0,
        "frames": 0,
        "errors": 0,
        "symbols": 0,
        "frame_ids": frame_ids,
    }

    with export_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            summary["rows"] += 1
            row_type = row["Type"]

            if row_type == "frame":
                summary["frames"] += 1
                frame_id = row["Frame ID"]
                if frame_id and frame_id != "-":
                    frame_ids[frame_id] += 1
            elif row_type == "error":
                summary["errors"] += 1
            elif row_type == "symbol":
                summary["symbols"] += 1

    return summary


def find_simulation_device(manager: automation.Manager) -> automation.DeviceDesc:
    devices = manager.get_devices(include_simulation_devices=True)

    for device in devices:
        if device.is_simulation:
            return device

    raise RuntimeError("No Logic 2 simulation device was reported by the automation API.")


def validate_demo_summary(summary: dict[str, object]) -> None:
    frame_count = int(summary["frames"])
    frame_ids: Counter[str] = summary["frame_ids"]  # type: ignore[assignment]

    if frame_count == 0:
        raise RuntimeError("Demo capture produced zero decoded FlexRay frames.")

    if len(frame_ids) < 2:
        observed = ", ".join(frame_ids.keys()) if frame_ids else "none"
        raise RuntimeError(f"Demo capture only decoded one synthetic frame type. Observed frame IDs: {observed}")


def run_demo_capture(
    manager: automation.Manager,
    args: argparse.Namespace,
    output_dir: Path,
    manual_rerun_capture_path: Path,
) -> tuple[automation.Capture, automation.AnalyzerHandle, str]:
    device = find_simulation_device(manager)
    capture = manager.start_capture(
        device_id=device.device_id,
        device_configuration=automation.LogicDeviceConfiguration(
            enabled_digital_channels=[args.channel],
            digital_sample_rate=args.sample_rate,
        ),
        capture_configuration=automation.CaptureConfiguration(
            buffer_size_megabytes=args.buffer_size,
            capture_mode=automation.TimedCaptureMode(duration_seconds=args.duration),
        ),
    )
    capture.wait()

    add_flexray_analyzer(capture, args)

    if args.demo_pause_seconds > 0:
        print(
            f"First-pass demo analyzer added at {args.bitrate} bit/s. "
            f"Sleeping {args.demo_pause_seconds:.1f}s before continuing."
        )
        print("You can inspect Logic 2 now, and press Run manually during this pause if needed.")
        time.sleep(args.demo_pause_seconds)

    if args.manual_rerun:
        prepared_capture_path = manual_rerun_capture_path
        print(f"Analyzer added to the demo capture at {args.bitrate} bit/s.")
        print(f"Rerun the capture in the Logic 2 GUI, save it to {prepared_capture_path}, then press Enter here.")
        input()

        if prepared_capture_path.exists() is False:
            raise FileNotFoundError(f"Manual rerun capture not found: {prepared_capture_path}")
    else:
        prepared_capture_path = output_dir / "flexray_demo_auto.sal"
        try:
            capture.save_capture(str(prepared_capture_path))
        except automation.errors.InvalidRequestError as exc:
            if 'does not exist' not in str(exc):
                raise

            prepared_capture_path = manual_rerun_capture_path
            print("The demo capture was rerun in the GUI during the pause, so the original automation capture no longer exists.")
            print(f"Save the current Logic 2 capture to {prepared_capture_path}, then press Enter here.")
            input()

            if prepared_capture_path.exists() is False:
                raise FileNotFoundError(f"Fallback demo capture not found: {prepared_capture_path}") from exc

    prepared_capture_for_api = create_capture_without_saved_analyzers(
        prepared_capture_path,
        output_dir / f"{prepared_capture_path.stem}_api{prepared_capture_path.suffix}",
    )

    close_capture_if_present(capture)
    capture = manager.load_capture(str(prepared_capture_for_api))
    analyzer = add_flexray_analyzer(capture, args)

    source_description = (
        f"Demo capture:  {prepared_capture_path}\n"
        f"API capture:   {prepared_capture_for_api}\n"
        f"Sample rate:   {args.sample_rate}\n"
        f"Bit rate:      {args.bitrate}"
    )

    if args.manual_rerun is False:
        source_description += "\nFlow:          automated two-pass demo setup"

    return capture, analyzer, source_description


def main() -> int:
    root = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser(description="Validate the FlexRay analyzer against a Logic 2 capture or demo simulation.")
    parser.add_argument("--logic2", help="Path to the Logic 2 executable, .app bundle, or AppImage.")
    parser.add_argument("--mode", choices=["capture", "demo"], default="capture", help="Load a .sal capture or record from the Logic 2 simulation device.")
    parser.add_argument("--capture", default=str(root / "assets" / "SP2018_FlexRay.sal"), help="Path to the .sal capture.")
    parser.add_argument("--output-dir", default=str(root / "build" / "automation"), help="Directory for exported CSV files.")
    parser.add_argument("--analyzers-dir", default=str(root / "build" / "Analyzers"), help="Directory containing libFlexRayAnalyzer.so.")
    parser.add_argument("--channel", type=int, default=0, help="Digital channel index for the FlexRay signal.")
    parser.add_argument("--bitrate", type=int, default=10_000_000, choices=[2_500_000, 5_000_000, 10_000_000], help="FlexRay bit rate.")
    parser.add_argument(
        "--sample-rate",
        type=int,
        default=40_000_000,
        help="Digital sample rate for demo mode. Defaults to 40 MS/s to give the analyzer simulator more timing margin.",
    )
    parser.add_argument("--duration", type=float, default=0.010, help="Capture duration in seconds for demo mode.")
    parser.add_argument("--buffer-size", type=int, default=32, help="Capture buffer size in megabytes for demo mode.")
    parser.add_argument(
        "--demo-pause-seconds",
        type=float,
        default=2.0,
        help="Pause after the first demo analyzer is added so you can observe Logic 2 before the second-pass load/export. Set 0 to disable.",
    )
    parser.add_argument("--save-capture", action="store_true", help="Save the recorded demo capture as .sal in the output directory.")
    parser.add_argument(
        "--manual-rerun",
        action="store_true",
        help="For demo mode: pause after adding the analyzer so you can rerun the capture in the GUI, save it, and then export that saved capture.",
    )
    parser.add_argument(
        "--manual-rerun-capture",
        help="Path to the .sal file you will save from the GUI during --manual-rerun. Defaults to <output-dir>/flexray_demo_manual.sal.",
    )
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
    manual_rerun_capture_path = Path(args.manual_rerun_capture).expanduser().resolve() if args.manual_rerun_capture else output_dir / "flexray_demo_manual.sal"
    output_dir.mkdir(parents=True, exist_ok=True)

    if analyzers_dir.exists() is False:
        raise FileNotFoundError(f"Analyzer output directory not found: {analyzers_dir}")

    if sys.platform.startswith("linux"):
        os.environ.setdefault("APPIMAGE_EXTRACT_AND_RUN", "1")
    config_path = configure_custom_analyzers_path(analyzers_dir)

    manager = automation.Manager.launch(
        application_path=str(logic2_path),
        connect_timeout_seconds=args.timeout,
        port=args.port,
    )

    capture: automation.Capture | None = None

    try:
        if args.mode == "capture":
            capture = manager.load_capture(str(capture_path))
            source_description = f"Capture:       {capture_path}"
            analyzer = add_flexray_analyzer(capture, args)
        else:
            capture, analyzer, source_description = run_demo_capture(manager, args, output_dir, manual_rerun_capture_path)

            if args.save_capture:
                final_demo_capture_path = output_dir / "flexray_demo.sal"
                capture.save_capture(str(final_demo_capture_path))
                source_description += f"\nSaved capture: {final_demo_capture_path}"

        try:
            legacy_csv, table_csv, raw_dir = export_capture_data(capture, analyzer, output_dir, args.channel)
            summary = summarize_legacy_export(legacy_csv)

            if args.mode == "demo":
                validate_demo_summary(summary)

            top_ids = ", ".join(f"{frame_id} x{count}" for frame_id, count in summary["frame_ids"].most_common(8)) or "-"

            print(f"Logic 2 binary: {logic2_path}")
            print(f"Config file:    {config_path}")
            print(f"Analyzers dir:  {analyzers_dir}")
            print(source_description)
            print(f"Legacy export:  {legacy_csv}")
            print(f"Data table:     {table_csv}")
            print(f"Raw export:     {raw_dir / 'digital.csv'}")
            print(f"Decoded rows:   {summary['rows']}")
            print(f"Frame rows:     {summary['frames']}")
            print(f"Symbol rows:    {summary['symbols']}")
            print(f"Error rows:     {summary['errors']}")
            print(f"Frame IDs:      {top_ids}")
        finally:
            close_capture_if_present(capture)
    finally:
        manager.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
