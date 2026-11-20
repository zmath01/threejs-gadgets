const test = require('node:test');
const assert = require('node:assert/strict');
const {
    STEP, TRIAL_STEPS, HORIZON, MAX_ACCELERATION, MPCController, advance, createTrial, referenceAt,
} = require('../drone-control.js');

const nearlyEquals = (a, b, tolerance = 1e-12) => assert.ok(Math.abs(a - b) < tolerance, `${a} ≠ ${b}`);

test('trial controls apply to the plant and share the same positional sensor noise', () => {
    const trial = createTrial(['pd', 'mpc'], true, null);
    const initial = trial.runs.map(run => run.state.slice());
    const initialEstimate = trial.runs[1].filter.state.slice();
    assert.equal(trial.step(), true);
    for (const [i, run] of trial.runs.entries()) {
        assert.ok(run.acceleration.some(x => x !== 0));
        advance(initial[i], run.acceleration);
        assert.deepEqual(run.state, initial[i]);
        assert.notDeepEqual(run.state, run.filter.state);
    }
    const [pd, mpc] = trial.runs;
    assert.notDeepEqual(pd.state, mpc.state);
    advance(initialEstimate, mpc.acceleration);
    for (let axis = 0; axis < 3; axis++) {
        nearlyEquals(pd.measurement[axis] - pd.state[axis], mpc.measurement[axis] - mpc.state[axis]);
        nearlyEquals(initialEstimate[axis], mpc.mpc.rollout[axis + 3]);
    }
    const ideal = createTrial(['mpc'], false, null);
    ideal.step();
    const [run] = ideal.runs;
    for (let axis = 0; axis < 3; axis++) nearlyEquals(run.state[axis], run.mpc.rollout[axis + 3]);
});

test('MPC rollout meets a finite-difference optimality check on unconstrained and boxed controls', () => {
    const state = Float64Array.of(0, 2.5, 0, 1, 0, -0.5);
    for (const waypoint of [[0.7, 2.2, -0.3], [12, 2, -12]]) {
        const target = Array.from({length: HORIZON}, (_, i) => referenceAt(i + 1, waypoint));
        const controller = new MPCController();
        const command = controller.solve(state, target);
        const controls = controller.controls.slice();

        function cost(inputs) {
            const pose = state.slice();
            let sum = 0;
            for (let i = 0; i < HORIZON; i++) {
                const acceleration = inputs.subarray(3 * i, 3 * i + 3);
                advance(pose, acceleration);
                for (let axis = 0; axis < 3; axis++) {
                    sum += 8 * (pose[axis] - target[i][axis]) ** 2 + 0.2 * acceleration[axis] ** 2;
                }
            }
            return sum;
        }

        assert.ok(cost(controls) < cost(controls.map(() => 0)));
        const epsilon = 1e-4;
        for (let i = 0; i < controls.length; i++) {
            const value = controls[i];
            assert.ok(Math.abs(value) <= MAX_ACCELERATION);
            if (i < 3) assert.equal(command[i], value);
            controls[i] = value + epsilon;
            const plus = cost(controls);
            controls[i] = value - epsilon;
            const minus = cost(controls);
            controls[i] = value;
            const gradient = (plus - minus) / (2 * epsilon);
            const descent = Math.max(-MAX_ACCELERATION, Math.min(MAX_ACCELERATION, value - gradient));
            nearlyEquals(descent, value, 5e-5);
        }
    }
});

test('replays and solo runs produce the same results over the fixed trial', () => {
    function run(algorithms, filter, waypoint) {
        const trial = createTrial(algorithms, filter, waypoint);
        let previousPeak = 0;
        for (let i = 0; i < TRIAL_STEPS; i++) {
            assert.equal(trial.step(), true);
            assert.ok(trial.runs[0].peakError >= previousPeak);
            previousPeak = trial.runs[0].peakError;
        }
        assert.equal(trial.step(), false);
        assert.equal(trial.steps * STEP, 20);
        for (const result of trial.runs) {
            assert.ok(result.state.every(Number.isFinite));
            assert.ok(result.acceleration.every(value => Math.abs(value) <= MAX_ACCELERATION));
            if (filter) assert.ok(result.filter.positionVariance.every(value => value >= 0 && Number.isFinite(value)));
            else assert.equal(result.filter, null);
        }
        return trial.runs;
    }
    for (const [useFilter, waypoint] of [[true, null], [false, [2, 2, -3]]]) {
        const compared = run(['pd', 'mpc'], useFilter, waypoint);
        for (const result of compared) {
            const [solo] = run([result.algorithm], useFilter, waypoint);
            assert.deepEqual(solo, result);
        }
        for (const {algorithm, errorSquared, effortSquared} of compared) {
            console.log({algorithm, useFilter, waypoint,
                trackingRms: Math.sqrt(errorSquared / TRIAL_STEPS),
                accelerationRms: Math.sqrt(effortSquared / TRIAL_STEPS)});
        }
    }
});
