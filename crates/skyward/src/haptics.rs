use crate::waveform::{made, scaled, share, ticks, RATE, TICKS_RATE};
use anyhow::{anyhow, ensure, Context, Result};
use evdev::raw_stream::RawDevice;
use skyward_config::Strength;
use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// linux/input-event-codes.h
const EV_FF: u16 = 0x15;
const FF_PERIODIC: u16 = 0x51;
const FF_CUSTOM: u16 = 0x5d;

/// The ticks play at this share of the drive the driver gives samples at in
/// full, whatever the slider says: the 17 Ultra's own system plays them at
/// 3527 mV of the 8500 mV its device tree gives.
const TICKS_STRENGTH: f32 = 0.415;
/// How long the ticks stay loaded once their samples have run out, as the
/// phone's own system leaves them: the motor comes to rest on its own
/// before taking them away brakes it.
const TICKS_REST: Duration = Duration::from_millis(40);
/// Duration of a finite AW8697 mBack click.
const MBACK_CLICK_DURATION: Duration = Duration::from_millis(15);
/// Spacing between the two screen-off ticks on AW8697.
const MBACK_SECOND_TICK_DELAY: Duration = Duration::from_millis(55);
/// Maximum gain accepted by Meizu's AW8697 sysfs interface.
const AW8697_MAX_GAIN: f32 = 128.0;

/// The vibration motor, either played through Linux force-feedback events
/// or through Meizu's AW8697 sysfs interface (`/sys/class/meizu/motor`).
pub struct Haptics {
    backend: Backend,
    strength: Strength,
}

enum Backend {
    ForceFeedback(ForceFeedback),
    MeizuMotor(MeizuMotor),
}

struct ForceFeedback {
    dev: File,
    click: Click,
    ticks: Click,
    /// Whether the driver is given the strength in the samples it plays.
    scale_samples: bool,
    /// The click playing, by its slot and its count, until it is taken
    /// away; a click taken away early by the next is not taken away again.
    playing: Arc<Mutex<Option<(i16, u64)>>>,
    clicks: u64,
}

struct MeizuMotor {
    dir: PathBuf,
    lock: Arc<Mutex<()>>,
}

/// What a click is.
enum Click {
    /// A waveform the driver keeps, by its number. The driver says how long
    /// it plays as it loads it.
    Kept(i16),
    /// Samples made at the motor's resonant frequency, which the driver
    /// plays as they are, at `rate` a second.
    Samples { data: Vec<i8>, rate: u32 },
}

impl Haptics {
    /// Open the configured vibration motor.
    pub fn open(cfg: &skyward_config::Haptics) -> Result<Haptics> {
        let backend = match &cfg.sysfs {
            Some(dir) => Backend::MeizuMotor(MeizuMotor::open(Path::new(dir))?),
            None => Backend::ForceFeedback(ForceFeedback::open(cfg)?),
        };
        Ok(Haptics { backend, strength: cfg.strength })
    }

    /// One short click at the strength the slider's `position` gives, after
    /// taking away one still playing. Gives how long it plays.
    pub fn click(&mut self, position: f32) -> Result<Duration> {
        let s = share(&self.strength, position);
        match &mut self.backend {
            Backend::ForceFeedback(ff) => ff.start(false, s),
            Backend::MeizuMotor(motor) => motor.click(position, s),
        }
    }

    /// The ticks of the screen going off, as strong as the phone's own
    /// system plays them whatever the slider says. Gives how long they play.
    pub fn ticks(&mut self) -> Result<Duration> {
        match &mut self.backend {
            Backend::ForceFeedback(ff) => ff.start(true, TICKS_STRENGTH),
            Backend::MeizuMotor(motor) => motor.ticks(),
        }
    }
}

impl ForceFeedback {
    fn open(cfg: &skyward_config::Haptics) -> Result<Self> {
        let (path, dev) = motor()?;
        let hz = resonance(&cfg.resonance)?;
        let click = match cfg.click {
            Some(id) => {
                eprintln!("haptics {}: clicks with waveform {id}, ticks made for a motor swinging at {hz} Hz", path.display());
                Click::Kept(id)
            }
            None => {
                eprintln!("haptics {}: clicks and ticks made for a motor swinging at {hz} Hz", path.display());
                Click::Samples { data: made(hz), rate: RATE }
            }
        };
        let ticks = Click::Samples { data: ticks(hz), rate: TICKS_RATE };
        Ok(ForceFeedback { dev, click, ticks, scale_samples: cfg.scale_samples, playing: Arc::new(Mutex::new(None)), clicks: 0 })
    }

    /// Play the ticks, or else the click, at `share` of its full strength,
    /// after taking away what still plays.
    fn start(&mut self, ticks: bool, share: f32) -> Result<Duration> {
        let mut playing = self.playing.lock().expect("no thread panics while it holds the click playing");
        if let Some((slot, _)) = playing.take() {
            erase(&self.dev, slot).context("take away the click still playing")?;
        }
        let effect = if ticks { &self.ticks } else { &self.click };
        let loaded = upload(&self.dev, effect, share, self.scale_samples).context("load the click")?;
        play(&mut self.dev, loaded.slot).context("play the click")?;
        self.clicks += 1;
        let this = (loaded.slot, self.clicks);
        *playing = Some(this);
        drop(playing);
        let dev = self.dev.try_clone().context("share the motor")?;
        let length = loaded.length + if ticks { TICKS_REST } else { Duration::ZERO };
        // Timed from now: a thread may wait a while before it first runs.
        let until = Instant::now() + length;
        let playing = self.playing.clone();
        std::thread::spawn(move || {
            std::thread::sleep(until.saturating_duration_since(Instant::now()));
            let mut playing = playing.lock().expect("no thread panics while it holds the click playing");
            if *playing == Some(this) {
                *playing = None;
                if let Err(e) = erase(&dev, this.0) {
                    eprintln!("take away the click: {e}");
                }
            }
        });
        Ok(length)
    }
}

impl MeizuMotor {
    fn open(dir: &Path) -> Result<Self> {
        ensure!(dir.join("set_mback").exists(), "{} has no set_mback control", dir.display());
        ensure!(dir.join("gain").exists(), "{} has no gain control", dir.display());
        let _ = write_sysfs(&dir.join("enable"), "0");
        let _ = write_sysfs(&dir.join("cont"), "0");
        if ram_num(dir) == Some(0) {
            let _ = write_sysfs(&dir.join("ram_update"), "1");
        }
        eprintln!("haptics {}: Meizu AW8697 mBack motor", dir.display());
        Ok(Self { dir: dir.to_path_buf(), lock: Arc::new(Mutex::new(())) })
    }

    fn click(&self, position: f32, share: f32) -> Result<Duration> {
        let level = mback_level(position);
        let gain = aw8697_gain(share);
        let _guard = self.lock.lock().expect("no thread panics while holding the motor lock");
        trigger_mback(&self.dir, level, gain)?;
        Ok(MBACK_CLICK_DURATION)
    }

    fn ticks(&self) -> Result<Duration> {
        let first_gain = aw8697_gain(TICKS_STRENGTH);
        let second_gain = aw8697_gain(TICKS_STRENGTH * 0.79);
        {
            let _guard = self.lock.lock().expect("no thread panics while holding the motor lock");
            trigger_mback(&self.dir, 0, first_gain)?;
        }
        let dir = self.dir.clone();
        let lock = self.lock.clone();
        std::thread::spawn(move || {
            std::thread::sleep(MBACK_SECOND_TICK_DELAY);
            let _guard = lock.lock().expect("no thread panics while holding the motor lock");
            if let Err(e) = trigger_mback(&dir, 0, second_gain) {
                eprintln!("haptics second tick: {e:#}");
            }
        });
        Ok(MBACK_SECOND_TICK_DELAY + MBACK_CLICK_DURATION)
    }
}

fn mback_level(position: f32) -> u8 {
    let p = position.clamp(0.0, 1.0);
    if p < 0.34 {
        0
    } else if p > 0.67 {
        2
    } else {
        1
    }
}

fn aw8697_gain(share: f32) -> u32 {
    (share.clamp(0.0, 1.0) * AW8697_MAX_GAIN).round().clamp(1.0, AW8697_MAX_GAIN) as u32
}

fn trigger_mback(dir: &Path, level: u8, gain: u32) -> Result<()> {
    // set_mback selects finite RAM wave 16/17/18 and starts it; it can reset
    // gain internally, so apply gain immediately after starting.
    write_sysfs(&dir.join("set_mback"), &format!("{level} 1"))?;
    write_sysfs(&dir.join("gain"), &gain.to_string())
}

fn ram_num(dir: &Path) -> Option<u32> {
    let text = fs::read_to_string(dir.join("ram_num")).ok()?;
    text.split_whitespace().rev().find_map(|tok| tok.parse::<u32>().ok())
}

fn write_sysfs(path: &Path, value: &str) -> Result<()> {
    let mut file = OpenOptions::new().write(true).open(path).with_context(|| format!("open {}", path.display()))?;
    file.write_all(value.as_bytes()).with_context(|| format!("write {value:?} to {}", path.display()))
}

/// Play the configured click once at the slider's `position`, or else the
/// ticks of the screen going off, saying how strongly and for how long, for
/// trying out a motor.
pub fn probe(cfg: &skyward_config::Haptics, position: f32, ticks: bool) -> Result<()> {
    let mut haptics = Haptics::open(cfg)?;
    let (what, strength, length) = if ticks {
        ("ticks", TICKS_STRENGTH, haptics.ticks()?)
    } else {
        ("click", share(&cfg.strength, position), haptics.click(position)?)
    };
    // Once it has played out, what played goes with the device as the
    // command ends.
    std::thread::sleep(length);
    println!("played the {what} at {strength:.3} of the full strength, {} ms long", length.as_millis());
    Ok(())
}

/// The first input device under /dev/input that takes periodic
/// force-feedback effects, opened for writing.
fn motor() -> Result<(PathBuf, File)> {
    for (path, dev) in evdev::raw_stream::enumerate() {
        if takes_periodic(&dev) {
            let file = OpenOptions::new().read(true).write(true).open(&path).with_context(|| format!("open {}", path.display()))?;
            return Ok((path, file));
        }
    }
    Err(anyhow!("no input device takes force-feedback effects"))
}

fn takes_periodic(dev: &RawDevice) -> bool {
    dev.supported_ff().is_some_and(|ff| ff.contains(evdev::FFEffectCode::FF_PERIODIC))
}

/// The motor's resonant frequency in hertz, from the file the driver says
/// it in, in tenths of a hertz.
fn resonance(file: &str) -> Result<f32> {
    let text = fs::read_to_string(file).with_context(|| format!("read {file}"))?;
    let tenths: u32 = text.split_whitespace().next().and_then(|n| n.parse().ok()).with_context(|| format!("{file} holds {text:?}"))?;
    ensure!(tenths > 0, "{file} says the motor swings at 0 Hz");
    Ok(tenths as f32 / 10.0)
}

/// A click as the driver loaded it: the slot the kernel keeps it in, and
/// how long it plays.
struct Loaded {
    slot: i16,
    length: Duration,
}

/// Samples handed to the driver to play as they are, as its
/// `custom_fifo_data` lays them out.
#[repr(C)]
struct Samples {
    /// Unused by the driver.
    index: u32,
    length: u32,
    rate: u32,
    data: *const i8,
}

/// Load `click` at `share` of its full strength as a custom periodic
/// effect in a slot of its own. The driver drives the motor at the click's
/// highest voltage times the magnitude; where it is given the strength in
/// the samples it plays, `scale_samples`, they are made that share of
/// themselves and the magnitude is the full one.
fn upload(dev: &File, click: &Click, share: f32, scale_samples: bool) -> Result<Loaded> {
    let share = share.clamp(0.0, 1.0);
    // Kept until the driver has copied what the effect points to.
    let lighter = match click {
        Click::Samples { data, .. } if scale_samples => Some(scaled(data, share)),
        _ => None,
    };
    // The driver tells the two kinds apart by the length in bytes of what
    // the effect points to, and loads nothing for any other length, though
    // the upload still succeeds. For a waveform it keeps it reads the
    // waveform's number from the first of three values and writes how long
    // it plays into the other two, in whole seconds and then milliseconds.
    let mut kept: [i16; 3] = [0; 3];
    let mut samples = Samples { index: 0, length: 0, rate: RATE, data: std::ptr::null() };
    let (len, data) = match click {
        Click::Kept(id) => {
            kept[0] = *id;
            (std::mem::size_of_val(&kept), kept.as_mut_ptr())
        }
        Click::Samples { data, rate } => {
            samples.length = u32::try_from(data.len())?;
            samples.rate = *rate;
            samples.data = lighter.as_deref().unwrap_or(data).as_ptr();
            (std::mem::size_of::<Samples>(), (&mut samples as *mut Samples).cast::<i16>())
        }
    };
    let periodic = libc::ff_periodic_effect {
        waveform: FF_CUSTOM,
        period: 0,
        magnitude: if lighter.is_some() { i16::MAX } else { (share * f32::from(i16::MAX)).round() as i16 },
        offset: 0,
        phase: 0,
        envelope: libc::ff_envelope { attack_length: 0, attack_level: 0, fade_length: 0, fade_level: 0 },
        custom_len: len as u32,
        custom_data: data,
    };
    let mut effect = libc::ff_effect {
        type_: FF_PERIODIC,
        id: -1,
        direction: 0,
        trigger: libc::ff_trigger { button: 0, interval: 0 },
        replay: libc::ff_replay { length: 0, delay: 0 },
        u: [0; 4],
    };
    // The union holds the periodic effect; it is exactly as large.
    const _: () = assert!(std::mem::size_of::<libc::ff_periodic_effect>() == std::mem::size_of::<[u64; 4]>());
    effect.u = unsafe { std::mem::transmute::<libc::ff_periodic_effect, [u64; 4]>(periodic) };
    // EVIOCSFF: _IOW('E', 0x80, struct ff_effect)
    let size = std::mem::size_of::<libc::ff_effect>() as u32;
    let request = (1u32 << 30) | (size << 16) | (u32::from(b'E') << 8) | 0x80;
    if unsafe { libc::ioctl(dev.as_raw_fd(), request as _, &mut effect) } < 0 {
        return Err(io::Error::last_os_error().into());
    }
    let length = match click {
        Click::Kept(id) => {
            let [_, seconds, millis] = kept;
            let length = Duration::from_secs(u64::try_from(seconds)?) + Duration::from_millis(u64::try_from(millis)?);
            ensure!(!length.is_zero(), "the driver loaded waveform {id} without saying how long it plays");
            length
        }
        // The queue is full before the samples start playing.
        Click::Samples { data, rate } => Duration::from_secs_f64(data.len() as f64 / f64::from(*rate)),
    };
    Ok(Loaded { slot: effect.id, length })
}

/// Start the loaded effect in slot `id`.
fn play(dev: &mut File, id: i16) -> io::Result<()> {
    let event = libc::input_event { time: libc::timeval { tv_sec: 0, tv_usec: 0 }, type_: EV_FF, code: id as u16, value: 1 };
    let bytes = unsafe { std::slice::from_raw_parts((&event as *const libc::input_event).cast::<u8>(), std::mem::size_of::<libc::input_event>()) };
    dev.write_all(bytes)
}

/// Take the effect in slot `id` away, which stops it for good.
fn erase(dev: &File, id: i16) -> io::Result<()> {
    // EVIOCRMFF: _IOW('E', 0x81, int)
    let request = (1u32 << 30) | ((std::mem::size_of::<libc::c_int>() as u32) << 16) | (u32::from(b'E') << 8) | 0x81;
    if unsafe { libc::ioctl(dev.as_raw_fd(), request as _, libc::c_int::from(id)) } < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aw8697_level_and_gain_follow_slider() {
        assert_eq!(mback_level(0.0), 0);
        assert_eq!(mback_level(0.5), 1);
        assert_eq!(mback_level(1.0), 2);
        assert_eq!(aw8697_gain(0.0), 1);
        assert_eq!(aw8697_gain(0.5), 64);
        assert_eq!(aw8697_gain(1.0), 128);
    }

    #[test]
    fn meizu_motor_writes_mback_and_gain() {
        let dir = tempfile::tempdir().unwrap();
        for name in ["set_mback", "gain", "enable", "cont", "ram_update"] {
            fs::write(dir.path().join(name), "").unwrap();
        }
        fs::write(dir.path().join("ram_num"), "ram_num = 0\n").unwrap();
        let cfg = skyward_config::Haptics {
            click: None,
            resonance: String::new(),
            sysfs: Some(dir.path().to_str().unwrap().to_string()),
            strength: Strength { lightest: 0.25, strongest: 1.0 },
            scale_samples: false,
        };
        let mut haptics = Haptics::open(&cfg).unwrap();
        assert_eq!(fs::read_to_string(dir.path().join("enable")).unwrap(), "0");
        assert_eq!(fs::read_to_string(dir.path().join("cont")).unwrap(), "0");
        assert_eq!(fs::read_to_string(dir.path().join("ram_update")).unwrap(), "1");
        let len = haptics.click(0.5).unwrap();
        assert_eq!(len, MBACK_CLICK_DURATION);
        assert_eq!(fs::read_to_string(dir.path().join("set_mback")).unwrap(), "1 1");
        assert_eq!(fs::read_to_string(dir.path().join("gain")).unwrap(), "64");

        // When RAM waveforms are already loaded, ram_update is not triggered again.
        fs::write(dir.path().join("ram_num"), "ram_num = 20\n").unwrap();
        fs::write(dir.path().join("ram_update"), "").unwrap();
        let _ = Haptics::open(&cfg).unwrap();
        assert_eq!(fs::read_to_string(dir.path().join("ram_update")).unwrap(), "");
    }
}
