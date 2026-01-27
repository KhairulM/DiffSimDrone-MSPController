import argparse
import select
import sys
import termios
import time
import tty

import numpy as np
import onnxruntime as ort
import pyrealsense2 as rs


def resize_nearest(image: np.ndarray, target_h: int, target_w: int) -> np.ndarray:
    """Nearest-neighbor resize without extra deps."""
    h, w = image.shape
    y_idx = np.linspace(0, h - 1, target_h, dtype=np.int64)
    x_idx = np.linspace(0, w - 1, target_w, dtype=np.int64)
    return image[np.ix_(y_idx, x_idx)]


def preprocess_depth(depth_frame: rs.depth_frame, depth_scale: float) -> np.ndarray:
    depth_m = np.asanyarray(depth_frame.get_data()).astype(
        np.float32) * depth_scale

    h, w = depth_m.shape
    target_w = int(h * 4 / 3)
    if target_w < w:
        start = (w - target_w) // 2
        depth_m = depth_m[:, start:start + target_w]

    valid = depth_m > 0
    inv_depth = np.zeros_like(depth_m, dtype=np.float32)
    inv_depth[valid] = 1.0 / depth_m[valid]

    if valid.any():
        mean = inv_depth[valid].mean()
        std = inv_depth[valid].std()
        if std < 1e-6:
            std = 1.0
        inv_depth[valid] = (inv_depth[valid] - mean) / std

    resized = resize_nearest(inv_depth, 24, 32)
    # 2x2 max-pool keeps nearest point per grid
    pooled = resized.reshape(12, 2, 16, 2).max(axis=(1, 3))
    return pooled[np.newaxis, np.newaxis, :, :].astype(np.float32)


def enter_cbreak() -> list:
    """Put stdin into cbreak mode so a single keypress is read without Enter."""
    if not sys.stdin.isatty():
        return []
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    return settings


def restore_tty(settings: list) -> None:
    if settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def q_pressed() -> bool:
    """Return True if the user pressed 'q' (case-insensitive)."""
    if not sys.stdin.isatty():
        return False
    if select.select([sys.stdin], [], [], 0)[0]:
        char = sys.stdin.read(1)
        return char.lower() == "q"
    return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx_model", default="base.onnx",
                        help="Path to ONNX model")
    parser.add_argument("--frames", type=int, default=0,
                        help="Number of frames to process before exiting (0 = run until 'q' key)")
    parser.add_argument(
        "--obs",
        type=float,
        nargs="*",
        default=None,
        help="Optional observation vector values (will be zero-filled if omitted)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    session = ort.InferenceSession(args.onnx_model, providers=[
                                   "CPUExecutionProvider"])
    inputs = session.get_inputs()
    image_input = next(inp for inp in inputs if len(inp.shape) >= 4)
    obs_input = next((inp for inp in inputs if len(inp.shape) == 2), None)
    obs_dim = obs_input.shape[1] if obs_input else None

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    tty_settings = enter_cbreak()

    try:
        idx = 0
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            frame_start = time.time()
            image = preprocess_depth(depth_frame, depth_scale)
            feed = {image_input.name: image}

            if obs_input:
                obs = np.zeros((1, obs_dim), dtype=np.float32)
                if args.obs:
                    obs_vals = np.asarray(args.obs, dtype=np.float32)
                    if obs_vals.size != obs_dim:
                        raise ValueError(
                            f"Expected {obs_dim} observation values, got {obs_vals.size}")
                    obs[0, :] = obs_vals
                feed[obs_input.name] = obs

            infer_start = time.time()

            outputs = session.run(None, feed)

            end = time.time()
            infer_ms = (end - infer_start) * 1000.0
            total_ms = (end - frame_start) * 1000.0
            fps = 1000.0 / total_ms if total_ms > 0.0 else float("inf")

            shapes = []
            for out in outputs:
                shape = out.shape if hasattr(out, "shape") else type(out)
                shapes.append(shape)
            print(
                f"Frame {idx}: infer {infer_ms:.2f} ms | frame {total_ms:.2f} ms | {fps:.2f} FPS | outputs {shapes}")

            idx += 1
            if args.frames and idx >= args.frames:
                break
            if q_pressed():
                print("'q' pressed, stopping.")
                break
    finally:
        restore_tty(tty_settings)
        pipeline.stop()


if __name__ == "__main__":
    main()
