package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * The saved droop curve only earns its keep if it comes back exactly as it went out, and only if
 * every way it could come back <i>wrong</i> ends with it being thrown away instead.
 *
 * <p>That second half is the point of most of these: a curve measured on different gearing, or a
 * file half-written by a brownout, is worse than no file at all — the compensator will apply it with
 * full confidence to the first ball of a match. Every rejection path has to land on "no data, use
 * the physics seed", which is where the robot was before any of this existed.
 */
class FlywheelDroopStoreTest {

    private static final String SIGNATURE = "test-shooter-geometry";
    private static final int BINS = 4;

    private static FlywheelDroopStore storeAt(Path dir) {
        return new FlywheelDroopStore(dir.resolve("flywheel-droop.json"), true);
    }

    private static void saveAndWait(
            FlywheelDroopStore store, double[] deficits, int[] samples, int shots) {
        store.save(deficits, samples, shots, SIGNATURE);
        store.awaitWrites();
    }

    @Test
    void savedCurveComesBackUnchanged(@TempDir Path dir) {
        double[] deficits = {0.0, 2.5, 5.25, 8.125};
        int[] samples = {0, 3, 11, 2};
        saveAndWait(storeAt(dir), deficits, samples, 16);

        // A fresh store, as if the robot had been power-cycled.
        FlywheelDroopStore.Persisted read = storeAt(dir).load(SIGNATURE, BINS);

        assertNotNull(read, "a file just written should load");
        assertArrayEquals(deficits, read.deficitRps, 1e-9);
        assertArrayEquals(samples, read.samples);
        assertEquals(16, read.shotCount);
    }

    @Test
    void nothingSavedYetIsNotAnError(@TempDir Path dir) {
        assertNull(storeAt(dir).load(SIGNATURE, BINS));
    }

    @Test
    void aCurveFromDifferentGearingIsRefused(@TempDir Path dir) {
        saveAndWait(storeAt(dir), new double[] {1, 2, 3, 4}, new int[] {1, 1, 1, 1}, 4);

        // Exactly the case the signature exists for: the wheel this was measured on is not the
        // wheel on the robot now, so the numbers are confidently wrong rather than merely missing.
        assertNull(storeAt(dir).load("some-other-geometry", BINS));
    }

    @Test
    void aChangedBinLayoutIsRefused(@TempDir Path dir) {
        saveAndWait(storeAt(dir), new double[] {1, 2, 3, 4}, new int[] {1, 1, 1, 1}, 4);

        assertNull(storeAt(dir).load(SIGNATURE, BINS + 2));
    }

    @Test
    void aTruncatedFileIsRefusedRatherThanGuessedAt(@TempDir Path dir) throws IOException {
        Path file = dir.resolve("flywheel-droop.json");
        saveAndWait(storeAt(dir), new double[] {1, 2, 3, 4}, new int[] {1, 1, 1, 1}, 4);

        String whole = Files.readString(file);
        Files.writeString(file, whole.substring(0, whole.length() / 2));

        FlywheelDroopStore store = storeAt(dir);
        assertNull(store.load(SIGNATURE, BINS));
        assertFalse(store.wasLoaded());
    }

    @Test
    void clearingRemovesTheFile(@TempDir Path dir) {
        Path file = dir.resolve("flywheel-droop.json");
        FlywheelDroopStore store = storeAt(dir);
        saveAndWait(store, new double[] {1, 2, 3, 4}, new int[] {1, 1, 1, 1}, 4);
        assertTrue(Files.exists(file));

        store.clear();
        store.awaitWrites();

        assertFalse(Files.exists(file), "a cleared curve must not come back at the next boot");
        assertNull(storeAt(dir).load(SIGNATURE, BINS));
    }

    @Test
    void theLatestSaveWins(@TempDir Path dir) {
        FlywheelDroopStore store = storeAt(dir);
        saveAndWait(store, new double[] {1, 1, 1, 1}, new int[] {1, 1, 1, 1}, 4);
        saveAndWait(store, new double[] {9, 9, 9, 9}, new int[] {2, 2, 2, 2}, 8);

        FlywheelDroopStore.Persisted read = storeAt(dir).load(SIGNATURE, BINS);

        assertNotNull(read);
        assertArrayEquals(new double[] {9, 9, 9, 9}, read.deficitRps, 1e-9);
        assertEquals(8, read.shotCount);
    }

    @Test
    void simulationNeverTouchesTheFilesystem(@TempDir Path dir) {
        Path file = dir.resolve("flywheel-droop.json");
        FlywheelDroopStore store = new FlywheelDroopStore(file, false);

        store.save(new double[] {1, 2, 3, 4}, new int[] {1, 1, 1, 1}, 4, SIGNATURE);
        store.awaitWrites();

        assertFalse(Files.exists(file), "a sim run must not leave a curve for a real robot to load");
        assertNull(store.load(SIGNATURE, BINS));
    }

    @Test
    void geometrySignatureTracksEveryInputThatChangesWhatADeficitMeans() {
        String base = FlywheelDroopStore.signature(10.0, 80.0, 28.0 / 18.0, 0.1, 0.002, 0.145);

        assertEquals(base, FlywheelDroopStore.signature(10.0, 80.0, 28.0 / 18.0, 0.1, 0.002, 0.145));
        assertFalse(base.equals(
            FlywheelDroopStore.signature(5.0, 80.0, 28.0 / 18.0, 0.1, 0.002, 0.145)));
        assertFalse(base.equals(
            FlywheelDroopStore.signature(10.0, 90.0, 28.0 / 18.0, 0.1, 0.002, 0.145)));
        assertFalse(base.equals(
            FlywheelDroopStore.signature(10.0, 80.0, 1.0, 0.1, 0.002, 0.145)));
        assertFalse(base.equals(
            FlywheelDroopStore.signature(10.0, 80.0, 28.0 / 18.0, 0.12, 0.002, 0.145)));
        assertFalse(base.equals(
            FlywheelDroopStore.signature(10.0, 80.0, 28.0 / 18.0, 0.1, 0.003, 0.145)));
        assertFalse(base.equals(
            FlywheelDroopStore.signature(10.0, 80.0, 28.0 / 18.0, 0.1, 0.002, 0.2)));
    }
}
