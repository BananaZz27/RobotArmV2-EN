# Saving the Standby pose into config.py (works on a copy in tmp, never the real file)
import math
import shutil

import config
import pytest
from servo_calibrate import CONFIG_PY, save_standby


@pytest.fixture
def cfg(tmp_path):
    p = tmp_path / "config.py"
    shutil.copy(CONFIG_PY, p)
    return p


def test_save_standby_rewrites_line(cfg):
    save_standby([10, 20.04, 30, 40], cfg)
    text = cfg.read_text(encoding="utf-8")
    assert "STANDBY_Q = [10.0, 20.0, 30.0, 40.0]" in text
    assert text.count("STANDBY_Q =") == 1


@pytest.mark.parametrize("bad", [[0, 500, 0, 0], [0, math.nan, 0, 0]])
def test_save_standby_rejects_bad(cfg, bad):
    before = cfg.read_text(encoding="utf-8")
    with pytest.raises(ValueError):
        save_standby(bad, cfg)
    assert cfg.read_text(encoding="utf-8") == before


def test_default_standby_within_limits():
    assert len(config.STANDBY_Q) == len(config.JOINTS)
    for i in range(len(config.JOINTS)):
        assert config.Q_MIN[i] <= config.STANDBY_Q[i] <= config.Q_MAX[i]
