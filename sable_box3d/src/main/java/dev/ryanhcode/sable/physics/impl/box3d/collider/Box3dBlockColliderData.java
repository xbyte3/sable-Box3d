package dev.ryanhcode.sable.physics.impl.box3d.collider;

import org.joml.Vector3d;
import org.joml.Vector3dc;


import javax.swing.*;
import java.util.ArrayList;
import java.util.List;

public class Box3dBlockColliderData {

    public static final Box3dBlockColliderData EMPTY =
            new Box3dBlockColliderData(0,0,0,false);

    public final List<Box> boxes = new ArrayList<>();

    public final double friction;
    public final double volume;
    public final double restitution;
    public final boolean liquid;

    public record Box(Vector3d min, Vector3d max) {}

    public Box3dBlockColliderData(double friction, double volume, double restitution, boolean liquid) {
        this.friction = friction;
        this.volume = volume;
        this.restitution = restitution;
        this.liquid = liquid;
    }

    public void addBox(Vector3dc min, Vector3dc max) {
        this.boxes.add(new Box(new Vector3d(min), new Vector3d(max)));
    }
}