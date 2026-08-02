package frc.robot.utils.simulation;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 * The 2026 Rebuilt arena, with the stock hubs swapped for {@link CenterReturningHub}s so scored
 * fuel returns to the center field pile instead of just dropping a few inches behind the hub.
 *
 * <p>Also disables maple-sim's "efficiency mode", which by default spawns only a third of the
 * center pile and just one alliance's depot (~144 pieces instead of the real ~408) to keep the
 * physics engine's load down. This spawns the full, game-accurate piece count instead — expect
 * a real performance cost from dyn4j handling that many bodies.
 *
 * <p>Also passes {@code AddRampCollider = false} to the base constructor. By default
 * Arena2026Rebuilt treats the whole 47x217in ramp area in front of each hub as a solid,
 * impassable wall (since it can't simulate an actual 3D incline), which blocked driving over the
 * bump. This shrinks the collider down to just the 47x47in hub footprint itself, so the ramp/bump
 * area is open to drive over (still with no real bump physics — maple-sim just treats it as flat
 * ground there instead of a wall).
 */
public class Robot8793Arena extends Arena2026Rebuilt {
    public Robot8793Arena() {
        super(false);

        setEfficiencyMode(false);

        customSimulations.remove(blueHub);
        customSimulations.remove(redHub);

        blueHub = new CenterReturningHub(this, true);
        redHub = new CenterReturningHub(this, false);

        addCustomSimulation(blueHub);
        addCustomSimulation(redHub);
    }
}
