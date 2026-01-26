import time

import onnxruntime
import argparse
import numpy as np

argparse = argparse.ArgumentParser()
argparse.add_argument('--onnx_model', default='base.onnx')
args = argparse.parse_args()

if __name__ == "__main__":
    # Load the ONNX model
    ort_session = onnxruntime.InferenceSession(
        args.onnx_model, providers=['CPUExecutionProvider'])

    # Create a dummy input tensor
    dummy_input = {
        "image":  (1, 1, 12, 16),
        # Adjust the size according to your model's expected input
        "observation": (1, 10)
    }

    # Run the model with the dummy input
    ort_inputs = {k: onnxruntime.OrtValue.ortvalue_from_numpy(
        np.random.rand(*v).astype(np.float32)) for k, v in dummy_input.items()}

    start = time.time()
    action, hx = ort_session.run(None, ort_inputs)
    end = time.time()

    # Print the outputs
    print("Action:", action)
    print("Inference time:", end - start, "seconds")
