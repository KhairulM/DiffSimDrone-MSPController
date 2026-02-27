"""ONNX model loading and inference for navigation."""

from pathlib import Path
import numpy as np
import onnxruntime


class NavigationModel:
    """Handles ONNX model loading and inference for drone navigation."""

    def __init__(self, model_path: str):
        """
        Initialize ONNX model.

        Args:
            model_path: Path to ONNX model file. Defaults to base.onnx in project root.
        """
        if model_path is None:
            model_path = str(Path(__file__).parent.parent / "base.onnx")

        # Optimized session options for ARM CPU
        sess_options = onnxruntime.SessionOptions()
        sess_options.intra_op_num_threads = 4
        sess_options.inter_op_num_threads = 1
        sess_options.execution_mode = onnxruntime.ExecutionMode.ORT_SEQUENTIAL
        sess_options.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.session = onnxruntime.InferenceSession(
            model_path, sess_options=sess_options, providers=['CPUExecutionProvider'])

        # Hidden state for recurrent model
        self.hidden_state = None

    def reset_hidden_state(self):
        """Reset the recurrent hidden state."""
        self.hidden_state = None

    def run(self, image: np.ndarray, observation: np.ndarray) -> np.ndarray:
        """
        Run ONNX model inference.

        Args:
            image: Preprocessed depth image (1, 1, 12, 16)
            observation: Observation vector (1, 10)

        Returns:
            action: Model output (1, 6) = [a_pred (3), v_pred (3)]
        """
        ort_inputs = {
            "image": image,
            "observation": observation
        }

        outputs = self.session.run(None, ort_inputs)
        action = outputs[0]  # (1, 6)
        self.hidden_state = outputs[1] if len(outputs) > 1 else None

        return action

    def action_to_acceleration(
        self,
        action: np.ndarray,
        R: np.ndarray,
        g_std: np.ndarray
    ) -> np.ndarray:
        """
        Convert model action output to desired acceleration.

        Args:
            action: Model output (1, 6) = [a_pred (3), v_pred (3)]
            R: Rotation matrix (3, 3)
            g_std: Gravity vector in world frame (3,)

        Returns:
            Desired acceleration in world frame (3,)
        """
        act = (R @ action.reshape(2, 3).T).T  # Rotate to world frame
        a_pred = act[0, :]
        v_pred = act[1, :]
        acc = a_pred - v_pred
        acc[2] += g_std[2]  # Add gravity compensation in z-axis
        return acc
