import torch
import argparse

from torch import nn

# Validation code model
# class Model(nn.Module):
#     def __init__(self, dim_obs=9, dim_action=4) -> None:
#         super().__init__()
#         self.stem = nn.Sequential(
#             nn.Conv2d(1, 32, 2, 2, bias=False),  # 1, 12, 16 -> 32, 6, 8
#             # nn.BatchNorm2d(32),
#             nn.LeakyReLU(0.05),
#             nn.Conv2d(32, 64, 3, bias=False), #  32, 6, 8 -> 64, 4, 6
#             # nn.BatchNorm2d(64),
#             nn.LeakyReLU(0.05),
#             nn.Conv2d(64, 128, 3, bias=False), #  64, 4, 6 -> 128, 2, 4
#             # nn.BatchNorm2d(128),
#             nn.LeakyReLU(0.05),
#             nn.Flatten(),
#             nn.Linear(128*2*4, 192, bias=False),
#         )
#         self.dim_obs = dim_obs
#         self.observation_fc = nn.Linear(dim_obs, 192)

#         self.gru = nn.GRUCell(192, 192)
#         self.action_fc = nn.Linear(192, dim_action, bias=False)
#         self.activation = nn.LeakyReLU(0.05)

#     def forward(self, x: torch.Tensor, v, hx=None):
#         img_feat = self.stem(x)
#         x = self.activation(img_feat + self.observation_fc(v))
#         hx = self.gru(x, hx)
#         action = self.action_fc(self.activation(hx))
#         return action, hx
    
# # Training code model
class Model(nn.Module):
    def __init__(self, dim_obs=9, dim_action=4) -> None:
        """
        Initialize the neural network model for drone control.

        This model combines convolutional layers for image processing with a GRU cell
        for temporal sequence modeling, designed for drone control tasks.

        Args:
            dim_obs (int): Dimension of observation/state vector. Default is 9.
            dim_action (int): Dimension of action space. Default is 4.

        Attributes:
            stem (nn.Sequential): Convolutional feature extraction pipeline that processes
                1-channel input images through 3 conv layers and outputs a 192-dim vector.
            v_proj (nn.Linear): Projects observation vector to 192-dim space.
                Weights initialized to 0.5x normal values for stable training.
            gru (nn.GRUCell): Recurrent unit with 192-dim hidden state for temporal modeling.
            fc (nn.Linear): Final fully-connected layer mapping hidden state to action space.
                Weights initialized to 0.01x normal values for conservative action initialization.
            act (nn.LeakyReLU): Activation function applied in the model.

        Note:
            Weight scaling (0.5 for v_proj, 0.01 for fc) is used for careful initialization
            to prevent extreme outputs early in training and improve training stability.
        """
        super().__init__()
        self.stem = nn.Sequential(
            nn.Conv2d(1, 32, 2, 2, bias=False),  # 1, 12, 16 -> 32, 6, 8
            nn.LeakyReLU(0.05),
            nn.Conv2d(32, 64, 3, bias=False), #  32, 6, 8 -> 64, 4, 6
            nn.LeakyReLU(0.05),
            nn.Conv2d(64, 128, 3, bias=False), #  64, 4, 6 -> 128, 2, 4
            nn.LeakyReLU(0.05),
            nn.Flatten(),
            nn.Linear(128*2*4, 192, bias=False),
        )
        self.dim_obs = dim_obs
        self.v_proj = nn.Linear(dim_obs, 192)
        self.v_proj.weight.data.mul_(0.5)

        self.gru = nn.GRUCell(192, 192)
        self.fc = nn.Linear(192, dim_action, bias=False)
        self.fc.weight.data.mul_(0.01)
        self.act = nn.LeakyReLU(0.05)

    def reset(self):
        pass

    def forward(self, x: torch.Tensor, v, hx=None):
        img_feat = self.stem(x)
        x = self.act(img_feat + self.v_proj(v))
        hx = self.gru(x, hx)
        act = self.fc(self.act(hx))
        return act, None, hx


argparse = argparse.ArgumentParser()
argparse.add_argument(
    '--net', default='/home/user/Documents/KAIST/Projects/DiffSimDrone/DiffPhysDrone/validation_code/high_speed_flight/src/e2e_planner_v2/base.pth')
argparse.add_argument('--no_odom', default=False, action='store_true')
args = argparse.parse_args()

if __name__ == "__main__":

    if args.no_odom:
        model = Model(4 + 3, 6).eval()
    else:
        model = Model(7 + 3, 6).eval()

    state_dict = torch.load(args.net, map_location='cpu')
    model.load_state_dict(state_dict)

    # Create a dummy input tensor
    dummy_input = torch.rand(1, 1, 12, 16), torch.rand(1, model.dim_obs)
    
    output_name = args.net.split('.')[0]

    # Export the model to ONNX format
    torch.onnx.export(
        model,
        dummy_input,
        f"{output_name}.onnx",
        input_names=["image", "observation"],
        output_names=["action", "hx"],
    )

    print(f"Model has been exported to {output_name}.onnx")
