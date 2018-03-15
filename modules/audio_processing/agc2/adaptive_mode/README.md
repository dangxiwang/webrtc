Run like this:

```python
import numpy as np
import matplotlib.pyplot as plt

def plot():
  dump_names = ['agc2_vad_probability', 'agc2_vad_rms_dbfs',
                'agc2_adaptive_level_estimate_dbfs',
                'agc2_adaptive_level_estimate_with_offset_dbfs',
                'agc2_adaptive_saturation_margin_db']
  dump_data = {n : np.fromfile(n + '_1-2.dat', np.float32) for n in dump_names}
  shape = np.shape(dump_data[dump_names[0]])
  for d in dump_data.values():
    assert np.shape(d) == shape

  f, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
  ax1.plot(dump_data['agc2_vad_probability'], label='agc2_vad_probability')
  ax1.legend()

  for d in dump_names[2:]:
    ax2.plot(dump_data[d], label=d)
  ax2.legend()
  ax3.plot(dump_data['agc2_vad_rms_dbfs'], label='agc2_vad_rms_dbfs')
  ax3.legend()
  plt.show()

plot()

```
