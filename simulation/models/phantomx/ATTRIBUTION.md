# Attribution

Vendored from https://github.com/HumaRobotics/phantomx_description
(commit: `2a94615e6f4ac1bac4f4c69e621765bad28048cc`),
licensed under the Simplified BSD License (see `LICENSE` in this
directory). This is a stand-in model for the project's own ArcheoHex
robot — see `../../docs/architecture.md` in the main repo.

Changes made to `urdf/phantomx.urdf`:

1. `package://` mesh URIs rewritten to relative paths (`../meshes/...`)
   so MuJoCo can resolve them without a ROS package index. (`scene.xml`'s
   own `<mesh file=...>` entries were separately adjusted to `meshes/...`,
   since `scene.xml` lives one directory up from `urdf/phantomx.urdf` and
   the same relative string does not resolve from both locations.)

2. Added a `<mujoco><compiler balanceinertia="true"
   discardvisual="false"/></mujoco>` block right after the `<robot>` tag.
   The upstream inertia tensors fail MuJoCo's physical-validity check
   (`A + B >= C`, raised on link `c1_rf`) without it; `balanceinertia`
   rebalances the already-invalid tensor rather than changing any mass or
   geometry properties. (`discardvisual="false"` is MuJoCo's default and
   is redundant here, but included for explicitness.)

3. Changed the `j_phantomx_attachment` joint from `type="fixed"` to
   `type="floating"`. This is a real kinematic change, not cosmetic: with
   a fixed joint, MuJoCo's URDF importer merges the whole robot into
   `worldbody` as static geometry (no `MP_BODY` body, no free joint, base
   welded to the world). The task's scene needs a genuine 6-DOF
   free-floating base (`MP_BODY` with a `type="free"` joint, so
   `model.njnt == 19` — 18 leg hinges + 1 free joint) rather than a
   robot welded in place, so the root attachment was changed to float
   instead.

Only the 8 meshes actually referenced by the URDF (`body`, `body_coll`,
`connect`, `connect_coll`, `thigh_l`, `thigh_l_coll`, `tibia_l`,
`tibia_l_coll`) were vendored — the upstream repo's `thigh_r`/`tibia_r`
meshes are unused because the URDF mirrors the left-side leg meshes for
the right-side legs via joint transforms.
