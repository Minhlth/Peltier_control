from pathlib import Path
from kconfiglib import Kconfig

p = Path('main/Kconfig.projbuild')
print('exists', p.exists())
conf = Kconfig(str(p), warn=False)
print('parsed ok')
print(conf.get_symbol('PELTIER_SELF_TEST_ENABLE').name)
print(conf.get_symbol('PELTIER_SELF_TEST_PULSE_MS').name)
