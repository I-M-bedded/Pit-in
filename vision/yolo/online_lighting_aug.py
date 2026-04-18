from __future__ import annotations

from pathlib import Path

import torch
import torch.nn.functional as F
from ultralytics.models.yolo.pose.train import PoseTrainer
from ultralytics.utils import DEFAULT_CFG


class OnlineLightingAugmentor:
    """
    Realistic train-time lighting corruption.

    - low-light: exposure drop + black-level lift + shot/read/chroma noise
    - backlight: veiling glare + local bloom + mild directional streak

    `_b` samples already contain Isaac Sim bloom/backlight, so their extra
    backlight probability is reduced.
    """

    def __init__(
        self,
        p_lowlight: float = 0.45,
        p_backlight: float = 0.28,
        p_backlight_b: float = 0.08,
        p_blur: float = 0.30,
        p_downup: float = 0.20,
    ):
        self.p_lowlight = float(p_lowlight)
        self.p_backlight = float(p_backlight)
        self.p_backlight_b = float(p_backlight_b)
        self.p_blur = float(p_blur)
        self.p_downup = float(p_downup)
        self._grid_cache: dict[tuple[int, int, str], tuple[torch.Tensor, torch.Tensor]] = {}

    def __call__(self, images: torch.Tensor, im_files=None) -> torch.Tensor:
        if images.ndim != 4 or images.shape[1] != 3:
            return images

        out = images.clone()
        batch_size = out.shape[0]
        for idx in range(batch_size):
            stem = ""
            if im_files is not None and idx < len(im_files):
                stem = Path(str(im_files[idx])).stem
            is_bloomed = stem.endswith("_b")

            img = out[idx]
            if torch.rand((), device=img.device) < self.p_lowlight:
                img = self._apply_low_light(img)

            backlight_p = self.p_backlight_b if is_bloomed else self.p_backlight
            if torch.rand((), device=img.device) < backlight_p:
                img = self._apply_backlight(img, is_bloomed=is_bloomed)

            out[idx] = img.clamp_(0.0, 1.0)
        return out

    def _apply_low_light(self, img: torch.Tensor) -> torch.Tensor:
        device, dtype = img.device, img.dtype

        gain = torch.empty((), device=device).uniform_(0.24, 0.52)
        gamma = torch.empty((), device=device).uniform_(1.45, 2.25)
        black = torch.empty((), device=device).uniform_(0.012, 0.040)
        color = torch.stack(
            [
                torch.empty((), device=device).uniform_(0.92, 0.99),
                torch.empty((), device=device).uniform_(0.95, 1.01),
                torch.empty((), device=device).uniform_(1.02, 1.10),
            ]
        ).to(dtype=dtype).view(3, 1, 1)

        img = (img.clamp_min(1e-6).pow(gamma) * gain + black).clamp(0.0, 1.0)
        img = (img * color).clamp(0.0, 1.0)

        shot_scale = torch.empty((), device=device).uniform_(0.05, 0.10)
        read_scale = torch.empty((), device=device).uniform_(0.008, 0.020)
        chroma_scale = torch.empty((), device=device).uniform_(0.010, 0.022)

        shot_std = torch.sqrt(img.clamp(0.0, 1.0) + 1e-6) * shot_scale
        img = img + torch.randn_like(img) * shot_std
        img = img + torch.randn_like(img) * read_scale

        chroma = torch.randn_like(img) * chroma_scale
        chroma[1].mul_(0.45)
        img = img + chroma
        img = img.clamp(0.0, 1.0)

        if torch.rand((), device=device) < self.p_blur:
            img = self._gaussian_blur(img, sigma=float(torch.empty((), device=device).uniform_(0.6, 1.1)))
        if torch.rand((), device=device) < self.p_downup:
            img = self._down_up_degrade(img, scale=float(torch.empty((), device=device).uniform_(0.55, 0.80)))
        return img.clamp(0.0, 1.0)

    def _apply_backlight(self, img: torch.Tensor, *, is_bloomed: bool) -> torch.Tensor:
        device, dtype = img.device, img.dtype
        _, h, w = img.shape
        yy, xx = self._meshgrid(h, w, device)

        from_left = bool(torch.rand((), device=device) < 0.5)
        sx = float(torch.empty((), device=device).uniform_(-0.12, 0.10) if from_left else torch.empty((), device=device).uniform_(0.90, 1.12))
        sy = float(torch.empty((), device=device).uniform_(0.18, 0.88))

        dx = xx - sx
        dy = yy - sy
        dist = torch.sqrt(dx * dx + dy * dy + 1e-6)

        veil = torch.exp(-(dist**2) / (2.0 * (0.42**2)))
        bloom = torch.exp(-(dist**2) / (2.0 * (0.16**2)))
        sign = 1.0 if from_left else -1.0
        streak_axis = dx * sign * 0.92 + dy * 0.18
        streak = torch.exp(-(streak_axis**2) / (2.0 * (0.030**2))) * torch.exp(-(dist**2) / (2.0 * (0.28**2)))

        if is_bloomed:
            veil_strength = float(torch.empty((), device=device).uniform_(0.08, 0.16))
            bloom_strength = float(torch.empty((), device=device).uniform_(0.06, 0.12))
            streak_strength = float(torch.empty((), device=device).uniform_(0.03, 0.07))
            desat_strength = float(torch.empty((), device=device).uniform_(0.05, 0.10))
        else:
            veil_strength = float(torch.empty((), device=device).uniform_(0.18, 0.34))
            bloom_strength = float(torch.empty((), device=device).uniform_(0.10, 0.22))
            streak_strength = float(torch.empty((), device=device).uniform_(0.06, 0.14))
            desat_strength = float(torch.empty((), device=device).uniform_(0.08, 0.16))

        glare = veil * veil_strength + bloom * bloom_strength + streak * streak_strength
        source = torch.tensor([1.0, 0.97, 0.92], device=device, dtype=dtype).view(3, 1, 1)

        img = img + glare.unsqueeze(0) * source
        gray = img.mean(dim=0, keepdim=True)
        sat_mix = (veil * desat_strength).clamp(0.0, 0.5).unsqueeze(0)
        img = img * (1.0 - sat_mix) + gray * sat_mix

        if torch.rand((), device=device) < 0.5:
            img = self._gaussian_blur(img, sigma=float(torch.empty((), device=device).uniform_(0.45, 0.9)))
        return img.clamp(0.0, 1.0)

    def _meshgrid(self, h: int, w: int, device: torch.device) -> tuple[torch.Tensor, torch.Tensor]:
        key = (h, w, str(device))
        if key not in self._grid_cache:
            yy = torch.linspace(0.0, 1.0, h, device=device)
            xx = torch.linspace(0.0, 1.0, w, device=device)
            self._grid_cache[key] = torch.meshgrid(yy, xx, indexing="ij")
        return self._grid_cache[key]

    @staticmethod
    def _gaussian_blur(img: torch.Tensor, sigma: float) -> torch.Tensor:
        radius = 2
        coords = torch.arange(-radius, radius + 1, device=img.device, dtype=img.dtype)
        kernel = torch.exp(-(coords**2) / (2 * sigma * sigma))
        kernel = kernel / kernel.sum()
        kernel2d = torch.outer(kernel, kernel).view(1, 1, 5, 5)
        kernel2d = kernel2d.expand(3, 1, 5, 5)
        return F.conv2d(img.unsqueeze(0), kernel2d, padding=2, groups=3).squeeze(0)

    @staticmethod
    def _down_up_degrade(img: torch.Tensor, scale: float) -> torch.Tensor:
        h, w = img.shape[1:]
        nh = max(16, int(h * scale))
        nw = max(16, int(w * scale))
        small = F.interpolate(img.unsqueeze(0), size=(nh, nw), mode="bilinear", align_corners=False)
        return F.interpolate(small, size=(h, w), mode="bilinear", align_corners=False).squeeze(0)


class LightingAugPoseTrainer(PoseTrainer):
    def __init__(self, cfg=None, overrides=None, _callbacks=None):
        overrides = dict(overrides or {})
        self.online_aug = bool(overrides.pop("online_aug", True))
        self.lowlight_p = float(overrides.pop("lowlight_p", 0.45))
        self.backlight_p = float(overrides.pop("backlight_p", 0.28))
        self.backlight_p_b = float(overrides.pop("backlight_p_b", 0.08))
        super().__init__(cfg or DEFAULT_CFG, overrides, _callbacks)
        self.lighting_augmentor = OnlineLightingAugmentor(
            p_lowlight=self.lowlight_p,
            p_backlight=self.backlight_p,
            p_backlight_b=self.backlight_p_b,
        )

    def preprocess_batch(self, batch: dict) -> dict:
        batch = super().preprocess_batch(batch)
        if self.online_aug and self.model is not None and getattr(self.model, "training", False):
            batch["img"] = self.lighting_augmentor(batch["img"], batch.get("im_file"))
        return batch
