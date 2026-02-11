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
    """Optimized preprocessing with minimal allocations."""
    depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32)
    depth_m *= depth_scale

    h, w = depth_m.shape
    target_w = int(h * 4 / 3)
    if target_w < w:
        start = (w - target_w) // 2
        depth_m = depth_m[:, start:start + target_w]

    # In-place operations where possible
    valid = depth_m > 0
    inv_depth = np.zeros_like(depth_m)
    np.divide(1.0, depth_m, out=inv_depth, where=valid)

    if valid.any():
        valid_values = inv_depth[valid]
        mean = valid_values.mean()
        std = valid_values.std()
        if std > 1e-6:
            inv_depth[valid] = (valid_values - mean) / std

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

    # Optimize ONNX session for ARM CPU
    sess_options = ort.SessionOptions()
    sess_options.intra_op_num_threads = 4  # Adjust based on MangoPi cores
    sess_options.inter_op_num_threads = 1
    sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

    session = ort.InferenceSession(
        args.onnx_model,
        sess_options=sess_options,
        providers=["CPUExecutionProvider"]
    )
    inputs = session.get_inputs()
    image_input = next(inp for inp in inputs if len(inp.shape) >= 4)
    obs_input = next((inp for inp in inputs if len(inp.shape) == 2), None)
    obs_dim = obs_input.shape[1] if obs_input else None

    # Cache output shapes once; they should remain constant
    output_shapes = [out.shape for out in session.get_outputs()]

    pipeline = rs.pipeline()
    config = rs.config()
    config.disable_all_streams()
    config.enable_stream(rs.stream.depth, 480, 270, rs.format.z16, 15)
    # config.enable_stream(rs.stream.depth, 256, 144, rs.format.z16, 90)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Warm-up: run a few inference passes to stabilize performance
    print("Warming up...")
    dummy_image = np.random.rand(1, 1, 12, 16).astype(np.float32)
    dummy_obs = np.zeros((1, obs_dim), dtype=np.float32)
    for _ in range(5):
        session.run(None, {image_input.name: dummy_image,
                    "observation": dummy_obs})
    print("Starting inference loop. Press 'q' to quit.\n")

    tty_settings = enter_cbreak()

    infer_times = []
    frame_times = []
    fps_arr = []

    try:
        idx = 0
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            frame_start = time.time()
            image = preprocess_depth(depth_frame, depth_scale)
            # simulate real observation input
            obs = np.random.rand(1, obs_dim).astype(np.float32)
            feed = {image_input.name: image, "observation": obs}

            infer_start = time.time()
            outputs = session.run(None, feed)

            end = time.time()
            infer_ms = (end - infer_start) * 1000.0
            total_ms = (end - frame_start) * 1000.0
            fps = 1000.0 / total_ms if total_ms > 0.0 else float("inf")

            infer_times.append(infer_ms)
            frame_times.append(total_ms)
            fps_arr.append(fps)

            print(
                f"Frame {idx}: infer {infer_ms:.2f} ms | frame {total_ms:.2f} ms | {fps:.2f} FPS | outputs {output_shapes}")

            idx += 1
            if (args.frames and idx >= args.frames) or q_pressed():
                print(
                    f"Average inference time: {np.mean(infer_times):.2f} ms")
                print(f"Average frame time: {np.mean(frame_times):.2f} ms")
                print(f"Average FPS: {np.mean(fps_arr):.2f}")
                break
    finally:
        restore_tty(tty_settings)
        pipeline.stop()


if __name__ == "__main__":
    main()
