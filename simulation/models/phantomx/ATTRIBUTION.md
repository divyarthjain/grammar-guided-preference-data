# Attribution

Vendored from https://github.com/HumaRobotics/phantomx_description
(commit: `2a94615e6f4ac1bac4f4c69e621765bad28048cc`),
licensed under the Simplified BSD License (see `LICENSE` in this
directory). This is a stand-in model for the project's own ArcheoHex
robot — see `../../docs/architecture.md` in the main repo.

Changes made: `urdf/phantomx.urdf` had its `package://` mesh URIs
rewritten to relative paths (`../meshes/...`) so MuJoCo can resolve them
without a ROS package index. Only the 8 meshes actually referenced by
the URDF (`body`, `body_coll`, `connect`, `connect_coll`, `thigh_l`,
`thigh_l_coll`, `tibia_l`, `tibia_l_coll`) were vendored — the upstream
repo's `thigh_r`/`tibia_r` meshes are unused because the URDF mirrors
the left-side leg meshes for the right-side legs via joint transforms.
