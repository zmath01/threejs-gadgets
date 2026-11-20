/* Fixed-step translational model for the browser comparison. State uses SI units. */
const droneControl = (() => {
    const STEP = 0.05;
    const TRIAL_STEPS = 400;
    const HORIZON = 16;
    const PATH_RATE = 0.35;
    const PATH_STEPS = 2 * Math.PI / (0.3 * PATH_RATE * STEP);
    const MAX_ACCELERATION = 4; // Per Cartesian axis, for both controllers.
    const DRAG = 0.98 ** (60 * STEP);
    const NOISE = 0.096; // Uniform position measurement error, ±0.096 m per axis.
    const INITIAL = [0, 2.5, 0, 1, 0.2, 0.5];
    const POSITION_WEIGHT = 8;
    const CONTROL_WEIGHT = 0.2;

    const limit = x => Math.max(-MAX_ACCELERATION, Math.min(MAX_ACCELERATION, x));

    function advance(state, acceleration) {
        for (let axis = 0; axis < 3; axis++) {
            state[axis + 3] = DRAG * (state[axis + 3] + STEP * acceleration[axis]);
            state[axis] += STEP * state[axis + 3];
        }
    }

    class KalmanFilter {
        constructor() {
            this.state = Float64Array.of(...INITIAL);
            this.state.fill(0, 3);
            // Three independent [position, velocity] filters.
            this.positionVariance = Float64Array.of(1, 1, 1);
            this.crossCovariance = new Float64Array(3);
            this.velocityVariance = Float64Array.of(1, 1, 1);
        }

        observe(measurement, acceleration) {
            advance(this.state, acceleration);
            const f = STEP * DRAG;
            const variance = NOISE ** 2 / 3;
            const processVelocityVariance = (0.8 * f) ** 2;
            for (let axis = 0; axis < 3; axis++) {
                const p = this.positionVariance[axis];
                const c = this.crossCovariance[axis];
                const v = this.velocityVariance[axis];
                const pp = p + 2 * f * c + f ** 2 * v + STEP ** 2 * processVelocityVariance;
                const pv = DRAG * (c + f * v) + STEP * processVelocityVariance;
                const vv = DRAG ** 2 * v + processVelocityVariance;
                const innovation = measurement[axis] - this.state[axis];
                const denominator = pp + variance;
                this.state[axis] += pp / denominator * innovation;
                this.state[axis + 3] += pv / denominator * innovation;
                this.positionVariance[axis] = pp * variance / denominator;
                this.crossCovariance[axis] = pv * variance / denominator;
                this.velocityVariance[axis] = vv - pv ** 2 / denominator;
            }
        }
    }

    class MPCController {
        constructor() {
            this.controls = new Float64Array(HORIZON * 3);
            this.rollout = new Float64Array((HORIZON + 1) * 3);
            this.acceleration = new Float64Array(3);
        }

        predict(state) {
            for (let axis = 0; axis < 3; axis++) {
                let velocity = state[axis + 3];
                this.rollout[axis] = state[axis];
                for (let step = 0; step < HORIZON; step++) {
                    const i = step * 3 + axis;
                    velocity = DRAG * (velocity + STEP * this.controls[i]);
                    this.rollout[i + 3] = this.rollout[i] + STEP * velocity;
                }
            }
        }

        solve(state, reference) {
            // Shift the previous feasible solution by one control interval.
            this.controls.copyWithin(0, 3);
            this.controls.fill(0, this.controls.length - 3);
            // Projected gradient with an O(horizon) adjoint. The Hessian infinity
            // norm for these settings is < 0.91, so the step 0.75 is smaller than 1/L.
            for (let iteration = 0; iteration < 34; iteration++) {
                this.predict(state);
                let maxChange = 0;
                for (let axis = 0; axis < 3; axis++) {
                    let dPosition = 0;
                    let dVelocity = 0;
                    for (let step = HORIZON - 1; step >= 0; step--) {
                        const i = step * 3 + axis;
                        dPosition += 2 * POSITION_WEIGHT * (this.rollout[i + 3] - reference[step][axis]);
                        dVelocity = DRAG * (dVelocity + STEP * dPosition);
                        const gradient = 2 * CONTROL_WEIGHT * this.controls[i] + STEP * dVelocity;
                        const next = limit(this.controls[i] - 0.75 * gradient);
                        maxChange = Math.max(maxChange, Math.abs(next - this.controls[i]));
                        this.controls[i] = next;
                    }
                }
                if (maxChange < 1e-5) break;
            }
            this.predict(state);
            this.acceleration.set(this.controls.subarray(0, 3));
            return this.acceleration;
        }
    }

    function referenceAt(step, waypoint) {
        if (waypoint) return [waypoint[0], waypoint[1], waypoint[2], 0, 0, 0];
        const t = step * STEP * PATH_RATE;
        return [
            5.5 * Math.sin(1.8 * t),
            2.2 + 1.2 * Math.sin(1.2 * t) * Math.cos(0.6 * t),
            3.85 * Math.sin(0.9 * t),
            5.5 * 1.8 * Math.cos(1.8 * t) * PATH_RATE,
            1.2 * (1.2 * Math.cos(1.2 * t) * Math.cos(0.6 * t) -
                0.6 * Math.sin(1.2 * t) * Math.sin(0.6 * t)) * PATH_RATE,
            3.85 * 0.9 * Math.cos(0.9 * t) * PATH_RATE,
        ];
    }

    function createTrial(algorithms, useKalman, waypoint) {
        const runs = algorithms.map(algorithm => ({
            algorithm,
            mpc: algorithm === 'mpc' ? new MPCController() : null,
            filter: useKalman ? new KalmanFilter() : null,
            state: Float64Array.of(...INITIAL),
            measurement: Float64Array.of(...INITIAL.slice(0, 3)),
            acceleration: new Float64Array(3),
            errorSquared: 0,
            peakError: 0,
            effortSquared: 0,
            estimateErrorSquared: 0,
        }));
        let seed = 42;
        let steps = 0;
        const noise = new Float64Array(3);
        const reference = Array.from({length: TRIAL_STEPS + HORIZON}, (_, i) => referenceAt(i + 1, waypoint));

        function step() {
            if (steps === TRIAL_STEPS) return false;
            for (let axis = 0; axis < 3; axis++) {
                seed = (Math.imul(1664525, seed) + 1013904223) >>> 0;
                noise[axis] = (seed / 2 ** 32 * 2 - 1) * NOISE;
            }
            const target = reference[steps];
            const future = runs.some(run => run.mpc) ? reference.slice(steps, steps + HORIZON) : null;
            for (const run of runs) {
                const state = run.filter?.state ?? run.state;
                if (run.mpc) {
                    run.acceleration.set(run.mpc.solve(state, future));
                } else {
                    for (let axis = 0; axis < 3; axis++) {
                        run.acceleration[axis] = limit(3.5 * (target[axis] - state[axis]) +
                            1.2 * (target[axis + 3] - state[axis + 3]));
                    }
                }

                advance(run.state, run.acceleration);
                for (let axis = 0; axis < 3; axis++) run.measurement[axis] = run.state[axis] + noise[axis];
                run.filter?.observe(run.measurement, run.acceleration);
                let errorSquared = 0;
                for (let axis = 0; axis < 3; axis++) {
                    errorSquared += (run.state[axis] - target[axis]) ** 2;
                    run.effortSquared += run.acceleration[axis] ** 2;
                    if (run.filter) run.estimateErrorSquared += (run.filter.state[axis] - run.state[axis]) ** 2;
                }
                run.errorSquared += errorSquared;
                run.peakError = Math.max(run.peakError, Math.sqrt(errorSquared));
            }
            steps++;
            return true;
        }

        return {runs, step, get steps() { return steps; }};
    }

    return {STEP, TRIAL_STEPS, HORIZON, PATH_STEPS, MAX_ACCELERATION, advance, MPCController, referenceAt, createTrial};
})();

if (typeof module === 'object' && module.exports) module.exports = droneControl;
