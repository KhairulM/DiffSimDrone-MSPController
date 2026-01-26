import time
import torch
from torch import nn


class Model(nn.Module):
    def __init__(self, dim_obs=9, dim_action=4) -> None:
        super().__init__()
        self.stem = nn.Sequential(
            nn.Conv2d(1, 32, 2, 2, bias=False),  # 1, 12, 16 -> 32, 6, 8
            # nn.BatchNorm2d(32),
            nn.LeakyReLU(0.05),
            nn.Conv2d(32, 64, 3, bias=False),  # 32, 6, 8 -> 64, 4, 6
            # nn.BatchNorm2d(64),
            nn.LeakyReLU(0.05),
            nn.Conv2d(64, 128, 3, bias=False),  # 64, 4, 6 -> 128, 2, 4
            # nn.BatchNorm2d(128),
            nn.LeakyReLU(0.05),
            nn.Flatten(),
            nn.Linear(128*2*4, 192, bias=False),
        )
        self.dim_obs = dim_obs
        self.observation_fc = nn.Linear(dim_obs, 192)

        self.gru = nn.GRUCell(192, 192)
        self.action_fc = nn.Linear(192, dim_action, bias=False)
        self.activation = nn.LeakyReLU(0.05)

    def forward(self, x: torch.Tensor, v, hx=None):
        img_feat = self.stem(x)
        x = self.activation(img_feat + self.observation_fc(v))
        hx = self.gru(x, hx)
        action = self.action_fc(self.activation(hx))
        return action, hx


if __name__ == "__main__":
    model = Model(10, 6)

    state_dict = torch.load('base.pth', map_location='cpu')
    model.load_state_dict(state_dict)

    # Create a dummy input tensor
    dummy_input = torch.rand(1, 1, 12, 16), torch.rand(1, model.dim_obs)

    dummy_image = torch.randn(1, 1, 12, 16)  # Adjust size as needed
    dummy_observation = torch.randn(1, 10)

    start = time.time()
    action, hx = model(dummy_image, dummy_observation)
    end = time.time()

    print("Action:", action)
    print("Inference time:", end - start, "seconds")
