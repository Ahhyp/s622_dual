"""M2.7 回归保护测试（2026-09-04，M2.7 freeze 配套工程保护项）。

守护三件曾出过/易出错的事：
1. ``arm:=left|right`` 真的被消费并写入 M2_MODE/M2_ARM（2026-09-03 曾失效，
   导致标定板挂错臂、左臂数据全废）；
2. ``T_LR = T_LC @ inv(T_RC)`` 组合方向不能写反（M2.7 验收⑤ 的唯一公式）；
3. 仿真 ``marker_size=0.1988`` 与真机 ``0.200`` 不能混用（config 值守卫）；
外加 GPT P1 建议：用人工构造 GT 验证 rot_only/trans_only 分解语义
（防止有人改脚本时把 GT 旋转/平移拼错）。
"""
import importlib.util
import os
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

from hand_eye_calibration import dual_base
from hand_eye_calibration.dual_base import (
    decompose_tlr_error,
    rotation_delta_deg,
    t_lr_cam,
    translation_error_mm,
)

PKG_ROOT = Path(__file__).resolve().parents[1]          # src/hand_eye_calibration
LAUNCH_FILE = (
    PKG_ROOT.parent / "gz_launch" / "launch"
    / "s622_global_handeye_sim.launch.py"
)


def _mkt(rpy_deg, xyz):
    m = np.eye(4)
    m[:3, :3] = R.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    m[:3, 3] = np.asarray(xyz, dtype=float)
    return m


def _random_transform(rng: np.random.Generator) -> np.ndarray:
    """通用非对称随机刚体变换（用于公式语义测试，避免对称 GT 的退化）。"""
    m = np.eye(4)
    m[:3, :3] = R.from_rotvec(rng.uniform(-1.0, 1.0, size=3)).as_matrix()
    m[:3, 3] = rng.uniform(-0.5, 0.5, size=3)
    return m


# ---- 合成 GT：与 URDF 一致的基座安装 + 任意相机位姿 ----
def _synthetic_gt():
    """构造满足 T_LR_GT = Rz(180°) + t(0.7,0,0) 的每臂 base_T_cam GT。"""
    # world -> base
    T_w_bl = _mkt((0, 0, 180), (0.35, 0.0, 0.0))
    T_w_br = _mkt((0, 0, 0), (-0.35, 0.0, 0.0))
    # world -> camera optical（任意静态位姿）
    T_w_cam = _mkt((0, 58, -90), (0.0, 0.5, 0.9))
    gt_left = np.linalg.inv(T_w_bl) @ T_w_cam
    gt_right = np.linalg.inv(T_w_br) @ T_w_cam
    # 校验组合 GT 平移 = (0.7, 0, 0)
    tlr = t_lr_cam(gt_left, gt_right)
    assert np.allclose(tlr[:3, 3], [0.7, 0, 0], atol=1e-9), "synthetic GT broken"
    return gt_left, gt_right


class TestTlrDirection(unittest.TestCase):
    """组合方向保护（代码公式层）。

    注意：T_LR GT = Rz(180°)+t(0.7,0,0) 是对合（逆=自身），L↔R 交换在"误差 vs GT"
    上不可辨——因此方向守护落在 t_lr_cam 公式语义 + derive 脚本参数映射上。
    """

    def test_formula_is_left_inv_right(self):
        rng = np.random.default_rng(3)
        for _ in range(5):
            L = _random_transform(rng)
            Rc = _random_transform(rng)
            expect = L @ np.linalg.inv(Rc)
            self.assertTrue(np.allclose(t_lr_cam(L, Rc), expect, atol=1e-9))
            # 不是 L @ R（漏 inv）也不是 R @ inv(L)（写反）
            self.assertFalse(np.allclose(t_lr_cam(L, Rc), L @ Rc, atol=1e-6))
            self.assertFalse(np.allclose(t_lr_cam(L, Rc), t_lr_cam(Rc, L), atol=1e-6))

    def test_gt_reproduces_zero_error(self):
        gl, gr = _synthetic_gt()
        err = translation_error_mm(t_lr_cam(gl, gr))
        self.assertLess(err, 1e-6, "GT 组合应零误差")

    def test_derived_script_uses_module_with_left_right_order(self):
        """derive 脚本必须调 dual_base.t_lr_cam 且参数顺序为 (left, right)。"""
        text = (PKG_ROOT / "scripts" / "derive_dual_base_from_camera.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("t_lr_cam(T_L_C, T_R_C)", text)
        self.assertNotIn("t_lr_cam(T_R_C, T_L_C)", text)
        self.assertNotIn("T_L_R_cam = T_L_C @ np.linalg.inv(T_R_C)", text)
        # T_L_C 必须来自 --left 文件、T_R_C 来自 --right 文件
        self.assertLess(text.index("left.transform.matrix()"),
                        text.index("right.transform.matrix()"))


class TestDecompose(unittest.TestCase):
    """GPT P1 分解语义：平移/旋转残差被正确隔离。"""

    def test_translation_only_lands_in_trans_only(self):
        gl, gr = _synthetic_gt()
        # 左臂平移 +1mm x（旋转精确）
        le = gl.copy()
        le[:3, 3] = le[:3, 3] + [0.001, 0, 0]
        d = decompose_tlr_error(le, gr, gl, gr)
        self.assertAlmostEqual(d["full"], 1.0, places=3)
        self.assertAlmostEqual(d["trans_only"], 1.0, places=3)
        self.assertAlmostEqual(d["rot_only"], 0.0, places=6)

    def test_rotation_only_lands_in_rot_only(self):
        gl, gr = _synthetic_gt()
        # 左臂绕 z 转 0.3°（平移精确）→ 旋转残差 × ~0.55m 相机向量
        le = gl.copy()
        le[:3, :3] = R.from_euler("z", 0.3, degrees=True).as_matrix() @ le[:3, :3]
        d = decompose_tlr_error(le, gr, gl, gr)
        self.assertAlmostEqual(d["rot_only"], d["full"], places=6)
        self.assertAlmostEqual(d["trans_only"], 0.0, places=6)
        # 0.3° ≈ 5.2mrad × ~0.55m(相机在左基座系 x 分量) ≈ 2-3mm 量级（非零）
        self.assertGreater(d["full"], 1.0)
        self.assertLess(d["full"], 6.0)

    def test_left_only_equals_full_when_right_gt(self):
        gl, gr = _synthetic_gt()
        le = gl.copy()
        le[:3, :3] = R.from_euler("x", 0.2, degrees=True).as_matrix() @ le[:3, :3]
        d = decompose_tlr_error(le, gr, gl, gr)
        self.assertAlmostEqual(d["full"], d["left_only"], places=9)
        self.assertAlmostEqual(d["right_only"], 0.0, places=9)


class TestMarkerSizeConfig(unittest.TestCase):
    """仿真 marker_size=0.1988 守卫（真机必须恢复 0.200，禁止混用）。"""

    def test_sim_eob_configs_use_0_1988(self):
        for arm in ("left", "right"):
            cfg = PKG_ROOT / "config" / f"global_eye_on_base_{arm}.yaml"
            self.assertTrue(cfg.exists(), f"missing {cfg.name}")
            doc = yaml.safe_load(cfg.read_text(encoding="utf-8"))
            params = doc["manual_calibration_assistant"]["ros__parameters"]
            size = params["vision_quality"]["marker_size_m"]
            self.assertAlmostEqual(
                size, 0.1988, places=6,
                msg=f"{cfg.name}: 仿真补偿值 0.1988 被改动——真机恢复 0.200 属预期，"
                    "但必须同步更新注释与 M2.7 文档（禁止仿真/真机混用）",
            )
            self.assertNotAlmostEqual(size, 0.20, places=6)

    def test_sim_config_documents_real_robot_note(self):
        cfg = PKG_ROOT / "config" / "global_eye_on_base_right.yaml"
        text = cfg.read_text(encoding="utf-8")
        self.assertIn("真机", text, "config 注释必须保留真机恢复 0.200 的提醒")


class TestM2ArmEnv(unittest.TestCase):
    """arm:= 参数必须被消费并写入 M2_MODE/M2_ARM（2026-09-03 bug 回归）。"""

    @classmethod
    def setUpClass(cls):
        self = cls
        if not LAUNCH_FILE.exists():
            raise unittest.SkipTest(f"gz launch not found: {LAUNCH_FILE}")
        try:
            spec = importlib.util.spec_from_file_location(
                "s622_global_handeye_sim_launch", LAUNCH_FILE
            )
            module = importlib.util.module_from_spec(spec)
            assert spec.loader is not None
            spec.loader.exec_module(module)
        except ModuleNotFoundError as exc:  # colcon test 环境有 ament；手动运行无则跳过
            raise unittest.SkipTest(f"launch deps unavailable in this env: {exc}")
        cls._apply = staticmethod(module._apply_arm_env)
        cls._default_arm = module.DEFAULT_ARM

    def _clean_env(self):
        for k in ("M2_MODE", "M2_ARM"):
            os.environ.pop(k, None)

    def test_apply_arm_left_right(self):
        self._clean_env()
        for arm in ("left", "right"):
            self._apply(arm)
            self.assertEqual(os.environ.get("M2_MODE"), arm)
            self.assertEqual(os.environ.get("M2_ARM"), arm)

    def test_empty_arm_falls_back_to_default(self):
        self._clean_env()
        self._apply("")
        self.assertEqual(os.environ.get("M2_ARM"), self._default_arm)
        self.assertEqual(os.environ.get("M2_MODE"), self._default_arm)


if __name__ == "__main__":
    unittest.main()
