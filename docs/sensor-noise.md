# Middle-sensor noise under the switching strategy

Roadmap item: investigate why the middle sensor (index 2) is the noisiest channel
under the variance-based switching strategy.

Reproduce with: `python analysis/sensor_noise.py`
(reads the CSVs in `data/`, prints the tables below, writes `gallery/sensor_noise.png`).

The 5-element IR array is indexed 0=left, 2=middle, 4=right. All numbers below
are per-sensor variance of the calibrated (0..1) reading. Medians are used for the
headline comparison because the raw logs are heavy-tailed (a few large excursions
inflate the mean); mean and median are both reported by the script.

## Per-sensor median variance

Straight line:

| method    |    s0    |    s1    |    s2    |    s3    |    s4    |
|-----------|----------|----------|----------|----------|----------|
| analog    | 0.000058 | 0.000039 | 0.000020 | 0.000036 | 0.000040 |
| digital   | 0.000015 | 0.000044 | 0.000050 | 0.000027 | 0.000016 |
| switching | 0.000012 | 0.000035 | 0.000039 | 0.000033 | 0.000009 |

Curve:

| method    |    s0    |    s1    |    s2    |    s3    |    s4    |
|-----------|----------|----------|----------|----------|----------|
| analog    | 0.000138 | 0.000150 | 0.000069 | 0.000378 | 0.000107 |
| digital   | 0.000013 | 0.000063 | 0.000043 | 0.000039 | 0.000015 |
| switching | 0.000015 | 0.000055 | 0.000088 | 0.000065 | 0.000016 |

## How much sensor 2 stands out

Ratio of the sensor-2 median variance to the average median variance of the four
non-middle sensors (0,1,3,4):

| method    | track         | sensor 2 | others avg | ratio  |
|-----------|---------------|----------|------------|--------|
| analog    | straight_line | 0.000020 | 0.000043   | 0.46x  |
| analog    | curve         | 0.000069 | 0.000193   | 0.36x  |
| digital   | straight_line | 0.000050 | 0.000025   | 1.96x  |
| digital   | curve         | 0.000043 | 0.000033   | 1.32x  |
| switching | straight_line | 0.000039 | 0.000022   | 1.75x  |
| switching | curve         | 0.000088 | 0.000038   | 2.34x  |

Reading: under analog, the middle sensor is the QUIETEST channel (0.36x-0.46x, the
edges dominate). Under switching, the middle sensor is the noisiest, at 1.75x-2.34x
the side-sensor average. Switching mirrors the digital profile, not the analog one.

## Evidence for each hypothesis

### (a) The middle sensor straddles the black/white boundary -> SUPPORTED

On-line readings (`data/results/{analog,digital}_BW_on_line.csv`) show the middle
sensor carries the largest signal while the robot follows the line:

- analog on-line mean per sensor: 0.100, 0.630, 0.930, 0.370, 0.110  (max at s2)
- digital on-line mean per sensor: 0.031, 0.337, 0.801, 0.157, 0.030  (max at s2)

The middle sensor sits over the line and reads near full-scale, with s1 and s3 in
the transition band. Whatever lateral jitter occurs, the steepest part of the
reflectance curve is under the middle and its immediate neighbours, so real motion
produces the largest reading swings there. This is a physical precondition, but it
is present for all three methods, so it alone does not explain why only switching
(and digital) show the middle-sensor spike.

### (b) Switching mixes two calibrations of the same sensor -> SUPPORTED (primary)

The switching log tags every data row with the mode that produced it (an
`AnalogVariance:` or `DigitalVariance:` label line precedes each row). Splitting the
switching rows by that label:

Straight line:
- analog-mode rows  (n=84):  0.000000, 0.000003, 0.000002, 0.000003, 0.000004
- digital-mode rows (n=191): 0.000025, 0.000081, 0.000095, 0.000083, 0.000022
- per-sensor |digital-analog| gap: max at sensor 2

Curve:
- analog-mode rows  (n=27):  0.000006, 0.000003, 0.000003, 0.000003, 0.000006
- digital-mode rows (n=203): 0.000015, 0.000059, 0.000096, 0.000081, 0.000016
- per-sensor |digital-analog| gap: max at sensor 2

Two facts stand out. First, switching spends most of its time in the digital branch
(191/275 straight, 203/230 curve), so the blended output is dominated by the digital
calibration. Second, the analog-mode rows are near-zero on every sensor, while the
digital-mode rows carry essentially all the variance, peaking at sensor 2. The middle
-sensor noise under switching is therefore the digital-calibration population showing
through, and the analog/digital gap is largest exactly at the middle sensor. Switching
inherits the digital branch's steep middle-sensor behaviour rather than averaging it
away.

### (c) The sensor-2 calibration scale amplifies noise -> PARTIALLY SUPPORTED

Two distinct things travel under "calibration scale":

Intrinsic resting noise is NOT elevated at sensor 2. Static-target calibration
variance (`data/calibration/*_Variance_*.csv`) at the middle sensor is average or
low:
- analog black median per sensor: 0.000015, 0.000012, 0.000008, 0.000010, 0.000011  (s2 lowest)
- digital black median per sensor: 0.000012, 0.000010, 0.000012, 0.000006, 0.000021  (s2 mid)

So the middle photodiode is not a noisy part at rest.

The normalization gain (span) IS largest at sensor 2. The black-minus-white contrast
per sensor (from the calibrated-readings files) is:
- analog span: 0.114, 0.602, 0.773, 0.348, 0.100  (max at s2)
- digital span: 0.039, 0.545, 0.910, 0.176, 0.014  (max at s2)

The middle sensor has the widest black/white span, and the digital span (0.910) is
steeper than the analog span (0.773). Calibration maps the raw reading onto 0..1 by
dividing by this span, so any given physical jitter is amplified most at the sensor
with the widest span, i.e. the middle, and more so under the digital calibration.
This explains why the same physical straddle (hypothesis a) turns into a variance
spike specifically under the digital-heavy switching mode, and not under analog.

## Conclusion

The data supports a combination of (b) and (a)/(c-as-gain), not intrinsic device
noise. Mechanism: the middle sensor physically straddles the line and carries the
largest, steepest signal (a). Its digital calibration has the widest normalization
span, so motion jitter is amplified most there (c, as scale gain, not resting noise).
Switching spends most of its time in the digital branch and copies that branch's
steep middle-sensor behaviour, so the blended output is noisiest at sensor 2
(b, the primary distinguishing factor versus analog). Pure analog does not show the
spike because its edge sensors dominate and its middle sensor is the quietest channel.

## Recommendation

1. Consistent calibration across modes. The switching blend should not import the
   digital branch's raw middle-sensor gain wholesale. Use one shared, gain-matched
   calibration for the middle sensor across analog and digital modes, or renormalize
   the digital span at sensor 2 toward the analog span (0.910 -> ~0.773) so mode
   swaps stop injecting a step change at the middle.

2. Per-sensor filtering on the middle sensor. Because sensor 2 legitimately carries
   the steepest, largest signal, apply extra low-pass or median smoothing to sensor 2
   (and, to a lesser degree, s1 and s3) before the variance-based switch decision.
   This targets the one channel whose variance drives the mode selection without
   over-smoothing the quiet edge sensors.

Either change should be validated by re-running `analysis/sensor_noise.py` and
confirming the switching sensor-2 ratio drops from ~1.75x-2.34x toward ~1.0x.
