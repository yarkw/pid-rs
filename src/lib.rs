#[derive(Debug)]
pub struct PidController {
    /// time step
    dt: f64,

    /// proportional gain
    kp: f64,

    /// integral gain
    ki: f64,

    /// derivative gain
    kd: f64,

    /// clamp values for anti-windup
    clamp_lo: f64,
    clamp_hi: f64,

    /// coefficient of simple exponential smoothing for differential term.
    /// valid range is [0,1].
    /// d[n] = smooth * (e[n] - e[n-1]) / dt + (1 - smooth) * d[n-1]
    ///    where d[n] is differential term and e[n] is error value.
    /// If smooth = 1, smoothing function is off.
    smooth: f64,

    /// proportional term
    p: f64,

    /// integral term
    i: f64,

    /// differential term
    d: f64,

    /// clamped or not
    unclamped: bool,

    /// previous error
    e_prev: f64,
}

impl PidController {
    pub fn new(dt: f64, clamp: (f64, f64)) -> Self {
        Self {
            dt,
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            clamp_lo: clamp.0,
            clamp_hi: clamp.1,
            smooth: 1.0,
            p: 0.0,
            i: 0.0,
            d: 0.0,
            unclamped: true,
            e_prev: 0.0,
        }
    }

    /// e: error value
    pub fn step(&mut self, e: f64) -> f64 {
        if self.unclamped {
            self.i += self.dt * e;
        }

        self.d = self.smooth * (e - self.e_prev) / self.dt + (1.0 - self.smooth) * self.d;

        let u = self.kp * e + self.ki * self.i + self.kd * self.d;

        self.unclamped = (self.clamp_lo < u) && (u < self.clamp_hi);
        self.e_prev = e;

        u
    }

    pub fn set_kp(&mut self, kp: f64) {
        if kp >= 0.0 {
            self.kp = kp;
        }
    }

    pub fn set_ki(&mut self, ki: f64) {
        if ki >= 0.0 {
            self.ki = ki;
        }
    }

    pub fn set_kd(&mut self, kd: f64) {
        if kd >= 0.0 {
            self.kd = kd;
        }
    }

    pub fn set_smooth(&mut self, smooth: f64) {
        if (0.0..=1.0).contains(&smooth) {
            self.smooth = smooth;
        }
    }

    pub fn dt(&self) -> f64 {
        self.dt
    }

    pub fn kp(&self) -> f64 {
        self.kp
    }

    pub fn ki(&self) -> f64 {
        self.ki
    }

    pub fn clamp_lo(&self) -> f64 {
        self.clamp_lo
    }

    pub fn clamp_hi(&self) -> f64 {
        self.clamp_hi
    }

    pub fn smooth(&self) -> f64 {
        self.smooth
    }

    pub fn p(&self) -> f64 {
        self.p
    }

    pub fn i(&self) -> f64 {
        self.i
    }

    pub fn d(&self) -> f64 {
        self.d
    }

    pub fn unclamped(&self) -> bool {
        self.unclamped
    }

    pub fn e_prev(&self) -> f64 {
        self.e_prev
    }
}
