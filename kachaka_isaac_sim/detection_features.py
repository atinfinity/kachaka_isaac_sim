"""Object detection features (GetObjectDetectionFeatures) from the front camera.

On the real robot these are the detector's feature maps. The custom detector
of pf-robotics/kachaka-transfer-learning trains a detection head on them and
runs it as predictor.onnx, whose inputs are fixed (create_onnx.py there):

    p4: (1, 96, 14, 25)   p5: (1, 96, 7, 13)   p6: (1, 96, 4, 7)

maps of FCOS levels 4-6 (strides 16, 32, 64) for a 224 x 398 image
(fcos_transfer/dataset.py: model_h, model_w).

The robot's detector network is not published, so the simulator makes maps
of the same shapes from another network: torchvision's MobileNetV3-Large
(ImageNet weights, downloaded once into the torch hub cache). At 224 x 398
its stride-16 block gives 112 x 14 x 25 and its last block 960 x 7 x 13; a
fixed random 1 x 1 projection takes each to 96 channels, and a stride-2 max
pool of p5 gives p6. A head trained on these maps works in the simulator,
not on the robot (and the other way round).

Names: the demo notebooks feed `feature.name.replace("_out", "")` to the
ONNX inputs, so the robot's names are taken to be p4_out, p5_out, p6_out
(assumed: from python/demos/run_custom_object_detection.ipynb).
"""

from __future__ import annotations

import threading

import numpy as np

MODEL_H, MODEL_W = 224, 398
CHANNELS = 96
SHAPES = {"p4_out": (1, CHANNELS, 14, 25), "p5_out": (1, CHANNELS, 7, 13), "p6_out": (1, CHANNELS, 4, 7)}
P4_BLOCK, P5_BLOCK = 12, 16  # MobileNetV3-Large `features` indices: stride 16 (112 ch), stride 32 (960 ch)


class FeatureExtractor:
    """Feature maps of an RGB image. torch is imported, and the weights
    loaded, on the first call; thread-safe."""

    def __init__(self, device: str | None = None, seed: int = 0):
        self.device = device
        self.seed = seed
        self.lock = threading.Lock()
        self.net = None

    def _load(self) -> None:
        import torch
        from torchvision.models import MobileNet_V3_Large_Weights, mobilenet_v3_large

        if self.device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        weights = MobileNet_V3_Large_Weights.IMAGENET1K_V2
        net = mobilenet_v3_large(weights=weights).features[:P5_BLOCK + 1].eval().to(self.device)
        g = torch.Generator().manual_seed(self.seed)
        c4, c5 = net[P4_BLOCK].out_channels, net[P5_BLOCK].out_channels
        self.proj4 = (torch.randn(CHANNELS, c4, 1, 1, generator=g) / c4 ** 0.5).to(self.device)
        self.proj5 = (torch.randn(CHANNELS, c5, 1, 1, generator=g) / c5 ** 0.5).to(self.device)
        t = weights.transforms()
        self.mean = torch.tensor(t.mean, device=self.device).view(1, 3, 1, 1)
        self.std = torch.tensor(t.std, device=self.device).view(1, 3, 1, 1)
        self.net = net

    def __call__(self, rgb: np.ndarray) -> dict[str, np.ndarray]:
        """`rgb`: H x W x 3 uint8. Returns float32 arrays of SHAPES."""
        import cv2
        import torch
        import torch.nn.functional as F

        with self.lock:
            if self.net is None:
                self._load()
            img = cv2.resize(rgb, (MODEL_W, MODEL_H), interpolation=cv2.INTER_AREA)
            with torch.inference_mode():
                x = torch.from_numpy(img).to(self.device).permute(2, 0, 1)[None].float() / 255.0
                x = (x - self.mean) / self.std
                for i, block in enumerate(self.net):
                    x = block(x)
                    if i == P4_BLOCK:
                        p4 = F.conv2d(x, self.proj4)
                p5 = F.conv2d(x, self.proj5)
                p6 = F.max_pool2d(p5, kernel_size=3, stride=2, padding=1)
            out = {"p4_out": p4, "p5_out": p5, "p6_out": p6}
            return {k: v.float().cpu().numpy() for k, v in out.items()}
