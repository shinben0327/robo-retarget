# Third-Party Notices

robo-retarget is MIT, Copyright (c) 2026 Jihwan Shin (`LICENSE`). This file covers the
third-party material redistributed **in this repository**, as those licences require.
Each keeps its own licence.

---

## GMR — General Motion Retargeting

| | |
|---|---|
| Location | `robo_retarget/gmr/` |
| Upstream | <https://github.com/YanjieZe/GMR> @ `25f56db` (2026-10-23) |
| Licence | MIT (`robo_retarget/gmr/LICENSE`) |
| Copyright | Copyright 2025 Yanjie Ze |
| Modified | Yes — `robo_retarget/gmr/README.md` lists the changes |

## hoi-retarget render styles

| | |
|---|---|
| Location | `robo_retarget/render.py` |
| Upstream | <https://github.com/leggedrobotics/hoi-retarget> `hoi_retarget/rendering/render_style.py` @ `b341dad` (2026-09-30) |
| Licence | BSD 3-Clause (below); `render.py` stays under it |
| Copyright | Copyright (c) 2026, ETH Zurich |
| Modified | Yes — condensed to robot-only rendering; the `daylight`, `night` and `slate` styles are new |

Licence text, verbatim from upstream:

<details>
<summary>BSD 3-Clause License</summary>

```
Copyright (c) 2026, ETH Zurich
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

See licenses/dependencies for license information of dependencies of this package.
```

</details>

## Unitree G1 robot description

| | |
|---|---|
| Location | `assets/robots/g1/` |
| Upstream | <https://github.com/YanjieZe/GMR> `assets/unitree_g1/g1_mocap_29dof.xml` @ `25f56db` (2026-10-23), from Unitree's G1 description |
| Licence | BSD 3-Clause (`assets/robots/g1/LICENSE`) |
| Copyright | Copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") |
| Modified | Renamed to `g1_29dof.xml`, `meshdir` pointed at `meshes/`; only the meshes the MJCF uses are kept |

## Unitree H1 robot description

| | |
|---|---|
| Location | `assets/robots/h1/` |
| Upstream | <https://github.com/google-deepmind/mujoco_menagerie> `unitree_h1/` @ `370c439` (2026-11-10) |
| Licence | BSD 3-Clause (`assets/robots/h1/LICENSE`) |
| Copyright | Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") |
| Modified | Massless `left_hand_tip` / `right_hand_tip` bodies added, `meshdir` pointed at `meshes/` |

## Booster K1, T1 and T2 robot descriptions and motions

| | |
|---|---|
| Location | `assets/robots/{k1,t1,t2}/`, `assets/motions/{k1,t1,t2}/` |
| Upstream | <https://github.com/BoosterRobotics/booster_assets> @ `12a9751` (2026-09-26) |
| Licence | BSD 3-Clause, one licence for the whole upstream repository (`assets/robots/{k1,t1,t2}/LICENSE`, `assets/motions/{k1,t1,t2}/LICENSE`) |
| Copyright | Copyright (c) 2025, BoosterRobotics |
| Modified | MJCFs: massless `left_hand_tip` / `right_hand_tip` bodies added (a comparable hand keypoint on every robot), `meshdir` pointed at `meshes/`. Motions: verbatim |

## ROBOTIS OP3 robot description

| | |
|---|---|
| Location | `assets/robots/op3/` |
| Upstream | <https://github.com/google-deepmind/mujoco_menagerie> `robotis_op3/` @ `370c439` (2026-11-10) |
| Licence | Apache License 2.0 (`assets/robots/op3/LICENSE`) |
| Copyright | ROBOTIS; MJCF by Google DeepMind |
| Modified | Massless `left_hand_tip` / `right_hand_tip` bodies added, `meshdir` pointed at `meshes/`; only the meshes the MJCF uses are kept |

## HighTorque Pi+ robot description

| | |
|---|---|
| Location | `assets/robots/pi_plus/` |
| Upstream | <https://github.com/bit-bots/bitbots_main> `src/bitbots_simulation/bitbots_mujoco_sim/xml/pi_plus.xml` @ `3b9f0c9` (2026-11-19) |
| Licence | MIT (`assets/robots/pi_plus/LICENSE`) |
| Copyright | Copyright (c) Hamburg Bit-Bots |
| Modified | Massless `left_hand_tip` / `right_hand_tip` bodies added, `meshdir` pointed at `meshes/`; the 20 meshes over 30k faces decimated to 30k (`tools/decimate_meshes.py`; each says so in its STL header), the other 15 verbatim |

## Unitree H2 robot description

| | |
|---|---|
| Location | `assets/robots/h2/` |
| Upstream | <https://github.com/unitreerobotics/unitree_mujoco> `unitree_robots/h2/h2_mujoco.xml` and its meshes @ `1eb6642` (2026-09-07) |
| Licence | BSD 3-Clause (`assets/robots/h2/LICENSE`) |
| Copyright | Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") |
| Modified | Renamed to `h2.xml`, `meshdir` pointed at `meshes` |

## ROBOTIS AI Sapiens K1 robot description

| | |
|---|---|
| Location | `assets/robots/ai_sapiens/` |
| Upstream | <https://github.com/ROBOTIS-GIT/ai_sapiens> `ai_sapiens_description/mujoco/k1_rev2/k1.xml` and `meshes/k1_rev2/` @ `0f1ca21` (2026-11-19) |
| Licence | Apache License 2.0 (`assets/robots/ai_sapiens/LICENSE`) |
| Copyright | ROBOTIS (the repository licence names no copyright holder) |
| Modified | Renamed to `ai_sapiens.xml`, `meshdir` pointed at `meshes/`; visual OBJ meshes converted to binary STL, and those over 1,500 faces decimated to ~35% of their faces (`tools/decimate_meshes.py`; each says so in its STL header); the MJCF names the `.stl` files and says it was modified; collision meshes verbatim |

## Hamburg Bit-Bots Wolfgang-OP robot description

| | |
|---|---|
| Location | `assets/robots/wolfgang/` |
| Upstream | <https://github.com/bit-bots/wolfgang_robot> `wolfgang_description/urdf/robot.urdf` and its meshes @ `b067cae` (2024-01-24) |
| Licence | MIT (`assets/robots/wolfgang/LICENSE`) |
| Copyright | Copyright (c) Hamburg Bit-Bots |
| Modified | Converted from URDF to MJCF with `tools/urdf_to_mjcf.py` (free root joint, geom groups) |

## unitree_rl_lab G1 motions

| | |
|---|---|
| Location | `assets/motions/g1/dance_102.csv`, `assets/motions/g1/gangnam_style.csv` |
| Upstream | <https://github.com/unitreerobotics/unitree_rl_lab> `source/unitree_rl_lab/unitree_rl_lab/tasks/mimic/robots/g1_29dof/` (`G1_Take_102.bvh_60hz.csv`, `G1_gangnam_style_V01.bvh_60hz.csv`) @ `4960b84` (2025-11-19) |
| Licence | Apache License 2.0 (below) |
| Copyright | Copyright 2024 The Isaac Lab Project Developers, as stated in the repository's licence (repository by Unitree Robotics) |
| Modified | Renamed only; contents verbatim |

Licence text, verbatim from upstream (`LICENCE`); upstream has no NOTICE file:

<details>
<summary>Apache License 2.0</summary>

```
                                 Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/

   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION

   1. Definitions.

      "License" shall mean the terms and conditions for use, reproduction,
      and distribution as defined by Sections 1 through 9 of this document.

      "Licensor" shall mean the copyright owner or entity authorized by
      the copyright owner that is granting the License.

      "Legal Entity" shall mean the union of the acting entity and all
      other entities that control, are controlled by, or are under common
      control with that entity. For the purposes of this definition,
      "control" means (i) the power, direct or indirect, to cause the
      direction or management of such entity, whether by contract or
      otherwise, or (ii) ownership of fifty percent (50%) or more of the
      outstanding shares, or (iii) beneficial ownership of such entity.

      "You" (or "Your") shall mean an individual or Legal Entity
      exercising permissions granted by this License.

      "Source" form shall mean the preferred form for making modifications,
      including but not limited to software source code, documentation
      source, and configuration files.

      "Object" form shall mean any form resulting from mechanical
      transformation or translation of a Source form, including but
      not limited to compiled object code, generated documentation,
      and conversions to other media types.

      "Work" shall mean the work of authorship, whether in Source or
      Object form, made available under the License, as indicated by a
      copyright notice that is included in or attached to the work
      (an example is provided in the Appendix below).

      "Derivative Works" shall mean any work, whether in Source or Object
      form, that is based on (or derived from) the Work and for which the
      editorial revisions, annotations, elaborations, or other modifications
      represent, as a whole, an original work of authorship. For the purposes
      of this License, Derivative Works shall not include works that remain
      separable from, or merely link (or bind by name) to the interfaces of,
      the Work and Derivative Works thereof.

      "Contribution" shall mean any work of authorship, including
      the original version of the Work and any modifications or additions
      to that Work or Derivative Works thereof, that is intentionally
      submitted to Licensor for inclusion in the Work by the copyright owner
      or by an individual or Legal Entity authorized to submit on behalf of
      the copyright owner. For the purposes of this definition, "submitted"
      means any form of electronic, verbal, or written communication sent
      to the Licensor or its representatives, including but not limited to
      communication on electronic mailing lists, source code control systems,
      and issue tracking systems that are managed by, or on behalf of, the
      Licensor for the purpose of discussing and improving the Work, but
      excluding communication that is conspicuously marked or otherwise
      designated in writing by the copyright owner as "Not a Contribution."

      "Contributor" shall mean Licensor and any individual or Legal Entity
      on behalf of whom a Contribution has been received by Licensor and
      subsequently incorporated within the Work.

   2. Grant of Copyright License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      copyright license to reproduce, prepare Derivative Works of,
      publicly display, publicly perform, sublicense, and distribute the
      Work and such Derivative Works in Source or Object form.

   3. Grant of Patent License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      (except as stated in this section) patent license to make, have made,
      use, offer to sell, sell, import, and otherwise transfer the Work,
      where such license applies only to those patent claims licensable
      by such Contributor that are necessarily infringed by their
      Contribution(s) alone or by combination of their Contribution(s)
      with the Work to which such Contribution(s) was submitted. If You
      institute patent litigation against any entity (including a
      cross-claim or counterclaim in a lawsuit) alleging that the Work
      or a Contribution incorporated within the Work constitutes direct
      or contributory patent infringement, then any patent licenses
      granted to You under this License for that Work shall terminate
      as of the date such litigation is filed.

   4. Redistribution. You may reproduce and distribute copies of the
      Work or Derivative Works thereof in any medium, with or without
      modifications, and in Source or Object form, provided that You
      meet the following conditions:

      (a) You must give any other recipients of the Work or
          Derivative Works a copy of this License; and

      (b) You must cause any modified files to carry prominent notices
          stating that You changed the files; and

      (c) You must retain, in the Source form of any Derivative Works
          that You distribute, all copyright, patent, trademark, and
          attribution notices from the Source form of the Work,
          excluding those notices that do not pertain to any part of
          the Derivative Works; and

      (d) If the Work includes a "NOTICE" text file as part of its
          distribution, then any Derivative Works that You distribute must
          include a readable copy of the attribution notices contained
          within such NOTICE file, excluding those notices that do not
          pertain to any part of the Derivative Works, in at least one
          of the following places: within a NOTICE text file distributed
          as part of the Derivative Works; within the Source form or
          documentation, if provided along with the Derivative Works; or,
          within a display generated by the Derivative Works, if and
          wherever such third-party notices normally appear. The contents
          of the NOTICE file are for informational purposes only and
          do not modify the License. You may add Your own attribution
          notices within Derivative Works that You distribute, alongside
          or as an addendum to the NOTICE text from the Work, provided
          that such additional attribution notices cannot be construed
          as modifying the License.

      You may add Your own copyright statement to Your modifications and
      may provide additional or different license terms and conditions
      for use, reproduction, or distribution of Your modifications, or
      for any such Derivative Works as a whole, provided Your use,
      reproduction, and distribution of the Work otherwise complies with
      the conditions stated in this License.

   5. Submission of Contributions. Unless You explicitly state otherwise,
      any Contribution intentionally submitted for inclusion in the Work
      by You to the Licensor shall be under the terms and conditions of
      this License, without any additional terms or conditions.
      Notwithstanding the above, nothing herein shall supersede or modify
      the terms of any separate license agreement you may have executed
      with Licensor regarding such Contributions.

   6. Trademarks. This License does not grant permission to use the trade
      names, trademarks, service marks, or product names of the Licensor,
      except as required for reasonable and customary use in describing the
      origin of the Work and reproducing the content of the NOTICE file.

   7. Disclaimer of Warranty. Unless required by applicable law or
      agreed to in writing, Licensor provides the Work (and each
      Contributor provides its Contributions) on an "AS IS" BASIS,
      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
      implied, including, without limitation, any warranties or conditions
      of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
      PARTICULAR PURPOSE. You are solely responsible for determining the
      appropriateness of using or redistributing the Work and assume any
      risks associated with Your exercise of permissions under this License.

   8. Limitation of Liability. In no event and under no legal theory,
      whether in tort (including negligence), contract, or otherwise,
      unless required by applicable law (such as deliberate and grossly
      negligent acts) or agreed to in writing, shall any Contributor be
      liable to You for damages, including any direct, indirect, special,
      incidental, or consequential damages of any character arising as a
      result of this License or out of the use or inability to use the
      Work (including but not limited to damages for loss of goodwill,
      work stoppage, computer failure or malfunction, or any and all
      other commercial damages or losses), even if such Contributor
      has been advised of the possibility of such damages.

   9. Accepting Warranty or Additional Liability. While redistributing
      the Work or Derivative Works thereof, You may choose to offer,
      and charge a fee for, acceptance of support, warranty, indemnity,
      or other liability obligations and/or rights consistent with this
      License. However, in accepting such obligations, You may act only
      on Your own behalf and on Your sole responsibility, not on behalf
      of any other Contributor, and only if You agree to indemnify,
      defend, and hold each Contributor harmless for any liability
      incurred by, or claims asserted against, such Contributor by reason
      of your accepting any such warranty or additional liability.

   END OF TERMS AND CONDITIONS

   APPENDIX: How to apply the Apache License to your work.

      To apply the Apache License to your work, attach the following
      boilerplate notice, with the fields enclosed by brackets "[]"
      replaced with your own identifying information. (Don't include
      the brackets!)  The text should be enclosed in the appropriate
      comment syntax for the file format. We also recommend that a
      file or class name and description of purpose be included on the
      same "printed page" as the copyright notice for easier
      identification within third-party archives.

   Copyright 2024 The Isaac Lab Project Developers

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
```

</details>

## Unitree LAFAN1 retargeting dataset (G1 and H1 sample motions)

| | |
|---|---|
| Location | `assets/motions/g1/{dance1_subject2,fight1_subject2,jumps1_subject1,walk1_subject1}.csv`, `assets/motions/h1/walk1_subject1.csv` |
| Upstream | Unitree's *LAFAN1_Retargeting_Dataset* (HuggingFace, `unitreerobotics/LAFAN1_Retargeting_Dataset`; copied from the mirror `lvhaidong/LAFAN1_Retargeting_Dataset`) |
| Licence | **CC BY-NC-ND 4.0** (<https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode>; also `assets/motions/h1/LICENSE`), inherited from [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) |
| Copyright | LAFAN1: (c) 2018 Ubisoft Entertainment; retargeting: Unitree Robotics |
| Modified | No — five CSV files, verbatim |

These are **non-commercial** and must not be redistributed in modified form, which
includes motions retargeted from them: they are here verbatim, as test inputs, and outputs
made from them are for non-commercial use and should not be redistributed. The code and every
other asset are permissively licensed. The licensed material is provided as-is, without
warranties of any kind (Section 5 of the licence).
