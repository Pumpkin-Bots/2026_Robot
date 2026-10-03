package frc.robot.utils;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Keeps what {@link FlywheelDroopCompensator} has learned on the roboRIO's flash, so a curve paid
 * for in balls is not thrown away by a power cycle.
 *
 * <h2>Why bother</h2>
 *
 * <p>The learned curve is worth several dozen shots of practice — the compensator needs balls
 * through the wheel at a spread of speeds before its bins mean anything, and every reboot between
 * matches starts that over from the physics seed. The droop itself does not reset when the robot
 * does: it is a property of the wheel, the gearing, and the ball, all of which are the same after
 * the reboot as before it. Carrying the estimate across is simply not discarding a measurement that
 * is still true.
 *
 * <h2>Where it goes</h2>
 *
 * <p>{@code /home/lvuser/flywheel-droop.json} on the robot — {@link Filesystem#getOperatingDirectory}.
 * Deliberately not the deploy directory: that is wiped and rewritten by every code push, and the
 * whole point is to survive one.
 *
 * <p>Writes are atomic (temp file, then rename), so a brownout part-way through a save leaves the
 * previous file intact rather than a half-written one. They also happen on a low-priority daemon
 * thread: the file is small, but flash on a roboRIO is not fast or predictable, and a 20 ms stall
 * inside the 200 Hz droop sampler would cost more than the save is worth.
 *
 * <h2>When a saved file is refused</h2>
 *
 * <p>A curve learned on a different mechanism is worse than no curve at all, because it is wrong
 * with confidence and the compensator will happily apply it to the first ball of a match. Each save
 * carries a {@link #signature()} built from every constant that changes what a learned deficit
 * means — gearing, effective diameter, inertia, ball mass, and the binning itself. Change any of
 * them and the file is discarded on the next boot and re-learned from scratch, which is the correct
 * response to "the shooter is not the shooter this was measured on".
 */
public final class FlywheelDroopStore {

    /** Bumped when the on-disk shape changes in a way older files cannot be read as. */
    private static final int SCHEMA_VERSION = 1;

    private static final String FILE_NAME = "flywheel-droop.json";
    private static final String TEMP_SUFFIX = ".tmp";

    private static final ObjectMapper MAPPER = new ObjectMapper();

    /** The on-disk shape. Public fields so Jackson maps it without annotations. */
    public static final class Persisted {
        public int schema = SCHEMA_VERSION;
        public String signature = "";
        public double[] deficitRps = new double[0];
        public int[] samples = new int[0];
        public int shotCount = 0;
        public long savedAtEpochMillis = 0L;
    }

    /**
     * Disabled in simulation. The compensator is inert there — maple-sim's projectiles leave at
     * exactly wheel speed, so there is no droop to learn — and a sim run writing a file that a real
     * robot would later load is a way to poison a real shot from a laptop.
     */
    private final boolean m_enabled;

    private final Path m_path;

    /** Latest payload waiting to be written; a newer save simply replaces an unstarted older one. */
    private final AtomicReference<String> m_pending = new AtomicReference<>(null);

    private ExecutorService m_writer = null;

    // Written by the writer thread, read by the robot thread publishing telemetry. Volatile so the
    // dashboard sees a completed save rather than a stale "idle" indefinitely.
    /** Human-readable account of the last load or save, for the dashboard. */
    private volatile String m_status = "idle";
    private volatile boolean m_loaded = false;

    public FlywheelDroopStore() {
        this(Filesystem.getOperatingDirectory().toPath().resolve(FILE_NAME), RobotBase.isReal());
    }

    /** Test seam: somewhere other than the roboRIO, switched on regardless of where it is running. */
    FlywheelDroopStore(Path path, boolean enabled) {
        m_path = path;
        m_enabled = enabled;
    }

    /** Blocks until every queued write has been flushed. Tests only — nothing on the robot waits. */
    void awaitWrites() {
        synchronized (this) {
            if (m_writer == null) {
                return;
            }
        }
        try {
            writer().submit(() -> { }).get();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (java.util.concurrent.ExecutionException e) {
            // Nothing to do: the write's own failure is already recorded in m_status.
        }
    }

    /**
     * Identifies the mechanism the numbers were measured on. Any change here invalidates every
     * saved bin, so this must name exactly the constants that change what a motor-RPS deficit means
     * — and nothing that does not, or the file gets thrown away over an unrelated tuning edit.
     *
     * <p>Not included: the detector's thresholds, the compensation gain, and the clamp. Those change
     * how the measurement is <i>taken</i> or <i>used</i>, not what the wheel physically does to a
     * ball, and re-learning the whole curve because someone nudged a rebound threshold would be a
     * needless cost.
     */
    public static String signature(
            double binWidthRps,
            double maxRps,
            double gearRatio,
            double effectiveDiameterMeters,
            double rotorMoiKgM2,
            double fuelMassKg) {
        return String.format(
            "w%.3f/m%.3f/g%.5f/d%.5f/i%.7f/b%.4f",
            binWidthRps, maxRps, gearRatio, effectiveDiameterMeters, rotorMoiKgM2, fuelMassKg);
    }

    /**
     * Reads the saved curve back, or returns {@code null} if there is nothing usable to read.
     *
     * <p>Every failure — missing file, unreadable JSON, wrong schema, wrong mechanism, wrong array
     * length — lands in the same place: no data, start from the physics seed. That is the same state
     * the robot was in before any of this existed, so there is no failure mode here that is worse
     * than not having it.
     *
     * @param expectedSignature the current mechanism's {@link #signature}
     * @param expectedBins      how many bins the compensator is expecting
     */
    public Persisted load(String expectedSignature, int expectedBins) {
        if (!m_enabled) {
            m_status = "simulation - not loaded";
            return null;
        }
        if (!Files.exists(m_path)) {
            m_status = "no saved file";
            return null;
        }

        Persisted read;
        try {
            read = MAPPER.readValue(m_path.toFile(), Persisted.class);
        } catch (IOException e) {
            m_status = "unreadable: " + e.getClass().getSimpleName();
            return null;
        }

        if (read == null || read.schema != SCHEMA_VERSION) {
            m_status = "wrong schema - discarded";
            return null;
        }
        if (!expectedSignature.equals(read.signature)) {
            m_status = "different shooter geometry - discarded";
            return null;
        }
        if (read.deficitRps == null || read.samples == null
                || read.deficitRps.length != expectedBins
                || read.samples.length != expectedBins) {
            m_status = "bin layout changed - discarded";
            return null;
        }
        for (double deficit : read.deficitRps) {
            if (!Double.isFinite(deficit)) {
                m_status = "corrupt values - discarded";
                return null;
            }
        }

        m_loaded = true;
        m_status = "loaded " + read.shotCount + " balls";
        return read;
    }

    /**
     * Queues a save. Returns immediately; the file is written on the background thread.
     *
     * <p>Serialisation happens here, on the caller's thread, so the bins are captured as they are
     * right now and the writer is handed an immutable string. Nothing about the compensator's state
     * is touched from the other thread.
     */
    public void save(double[] deficitRps, int[] samples, int shotCount, String signature) {
        if (!m_enabled) {
            return;
        }

        Persisted out = new Persisted();
        out.signature = signature;
        out.deficitRps = Arrays.copyOf(deficitRps, deficitRps.length);
        out.samples = Arrays.copyOf(samples, samples.length);
        out.shotCount = shotCount;
        out.savedAtEpochMillis = System.currentTimeMillis();

        String payload;
        try {
            payload = MAPPER.writerWithDefaultPrettyPrinter().writeValueAsString(out);
        } catch (IOException e) {
            m_status = "serialise failed: " + e.getClass().getSimpleName();
            return;
        }

        m_pending.set(payload);
        writer().execute(this::drain);
    }

    /** Deletes the saved file, so the next boot starts clean. */
    public void clear() {
        if (!m_enabled) {
            return;
        }
        m_pending.set(null);
        m_loaded = false;
        writer().execute(() -> {
            try {
                Files.deleteIfExists(m_path);
                m_status = "cleared";
            } catch (IOException e) {
                m_status = "clear failed: " + e.getClass().getSimpleName();
            }
        });
    }

    /** True if a saved curve was found and accepted at boot. */
    public boolean wasLoaded() {
        return m_loaded;
    }

    /** One line on what the store last did, for the dashboard. */
    public String status() {
        return m_status;
    }

    /** Writes whatever is pending. Coalescing means a queued task may find nothing left to do. */
    private void drain() {
        String payload = m_pending.getAndSet(null);
        if (payload == null) {
            return;
        }

        Path temp = m_path.resolveSibling(FILE_NAME + TEMP_SUFFIX);
        try {
            Files.writeString(temp, payload);
            try {
                Files.move(temp, m_path,
                    StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
            } catch (java.nio.file.AtomicMoveNotSupportedException e) {
                Files.move(temp, m_path, StandardCopyOption.REPLACE_EXISTING);
            }
            m_status = "saved";
        } catch (IOException e) {
            m_status = "save failed: " + e.getClass().getSimpleName();
        }
    }

    /**
     * The writer thread, created on first use. Daemon, so it never holds the JVM open, and lowest
     * priority, so it loses every scheduling argument it has with the robot loop.
     */
    private synchronized ExecutorService writer() {
        if (m_writer == null) {
            m_writer = Executors.newSingleThreadExecutor(runnable -> {
                Thread thread = new Thread(runnable, "FlywheelDroopStore");
                thread.setDaemon(true);
                thread.setPriority(Thread.MIN_PRIORITY);
                return thread;
            });
        }
        return m_writer;
    }
}
