.. _FilamentRendering:

Filament Rendering
------------------

MuJoCo includes a renderer based on `Filament <https://github.com/google/filament>`__, Google's real-time physically
based rendering engine. Filament is small and fast, runs on Linux, Windows, macOS, Android, iOS and the Web, and
supports the OpenGL, Vulkan, Metal and WebGL graphics backends. The Filament renderer powers :ref:`MuJoCo
Studio<Studio>` and :ref:`MuJoCo Live<Live>`.

.. attention::
   The Filament renderer is under active development. The APIs, render flags and code locations described in this
   chapter will change as the renderer matures.

The Filament renderer is enabled by setting ``MUJOCO_USE_FILAMENT=1`` in the CMake build configuration. This replaces
the OpenGL-based implementations of the ``mjr`` functions described in the :ref:`Rendering` section with Filament-based
ones, and makes the lower-level ``mjrf`` :ref:`types<tyFilamentRenderStructure>` and
:ref:`functions<FilamentRenderingApi>` available.

.. _fiPbr:

Physically based rendering
~~~~~~~~~~~~~~~~~~~~~~~~~~

The classic renderer implements the traditional Phong model: surface color is a sum of ambient, diffuse, specular and
emissive terms, and light intensities are dimensionless values tuned by eye. Physically based rendering (PBR) instead
describes surfaces with measurable properties -- base color, metallic and roughness -- and lights with photometric
units. Because material and lighting parameters are physical, assets retain their appearance across scenes and lighting
conditions, and effects like image-based lighting and realistic metals become possible. For an introduction to the
underlying theory, see Filament's `PBR documentation <https://google.github.io/filament/Filament.html>`__.

The Filament renderer supports both models. Materials and lights authored for the classic renderer are drawn with Phong
shading that matches the classic appearance, while physical materials and lights opt in to the PBR pipeline, as
described below.

Materials
^^^^^^^^^

A material is shaded with the PBR pipeline if it defines any physical property: a non-negative
:ref:`metallic<asset-material-metallic>` or :ref:`roughness<asset-material-roughness>` coefficient, or texture
:ref:`layers<material-layer>` with PBR roles (metallic, roughness, packed occlusion-roughness-metallic, normal,
emissive, occlusion). Materials with no physical properties are shaded with Phong.

Lighting
^^^^^^^^

Physical lighting is enabled by the lights themselves: if any light in the model has positive
:ref:`intensity<body-light-intensity>`, all lights are photometric, with intensities measured in candela. Lights of
:ref:`type<body-light-type>` ``image`` provide image-based lighting: the referenced texture illuminates the scene as an
environment map. A light's :ref:`bulbradius<body-light-bulbradius>` controls the softness of its shadows.

If no light has positive intensity, the model is treated as authored for the classic renderer: a default environment
light is added and default intensities are distributed among the model's lights. These defaults are set by the
``filament.fallback`` :ref:`render flags<fiFlags>`.

.. _fiFlags:

Render flags
~~~~~~~~~~~~

Renderer features are controlled by :ref:`custom<custom>` model elements: :ref:`numeric<custom-numeric>` and
:ref:`text<custom-text>` elements whose names begin with ``filament.`` are read by the renderer:

.. code-block:: xml

   <custom>
     <numeric name="filament.bloom.enabled" data="1"/>
     <numeric name="filament.fog.color" data="0.5 0.6 0.7"/>
     <text name="filament.cg.tone_mapping" data="filmic"/>
   </custom>

Flags correspond directly to Filament's rendering options; flags not specified in the model keep Filament's built-in
defaults, denoted -- in the tables below. Boolean flags are numerics taking values 0 or 1. Quality flags take integer
values 0 to 3, corresponding to Filament's low, medium, high and ultra quality levels. As the feature set stabilizes,
important flags will be promoted to first-class MJCF attributes.

Screen-space ambient occlusion:

.. list-table::
   :widths: 32 10 12 46
   :header-rows: 1

   * - Flag
     - Type
     - Default
     - Description
   * - ``filament.ao.enabled``
     - bool
     - 1
     - Enable screen-space ambient occlusion.
   * - ``filament.ao.quality``
     - quality
     - 3
     - Sampling quality.
   * - ``filament.ao.low_pass_filter``
     - quality
     - 3
     - Quality of the depth-aware blur filter.
   * - ``filament.ao.upsampling``
     - quality
     - 3
     - Quality of the occlusion buffer upsampling.
   * - ``filament.ao.bent_normals``
     - bool
     - 0
     - Compute bent normals for specular occlusion.
   * - ``filament.ao.ssct``
     - bool
     - --
     - Enable screen-space cone tracing.
   * - ``filament.ao.bilateral_threshold``
     - real
     - 0.5
     - Depth difference treated as an edge by the blur filter.

Bloom:

.. list-table::
   :widths: 32 10 12 46
   :header-rows: 1

   * - Flag
     - Type
     - Default
     - Description
   * - ``filament.bloom.enabled``
     - bool
     - --
     - Enable bloom.
   * - ``filament.bloom.strength``
     - real
     - --
     - Strength of the bloom effect, between 0 and 1.
   * - ``filament.bloom.dirt_strength``
     - real
     - --
     - Strength of the lens-dirt effect.
   * - ``filament.bloom.quality``
     - quality
     - --
     - Quality of the bloom passes.
   * - ``filament.bloom.resolution``
     - int
     - --
     - Resolution of the bloom's minor axis, in pixels.
   * - ``filament.bloom.levels``
     - int
     - --
     - Number of blur levels.

Color grading, applied in the listed order:

.. list-table::
   :widths: 32 10 12 46
   :header-rows: 1

   * - Flag
     - Type
     - Default
     - Description
   * - ``filament.cg.exposure``
     - real
     - 0
     - Exposure adjustment, in stops.
   * - ``filament.cg.temperature``
     - real
     - 0
     - White balance temperature.
   * - ``filament.cg.tint``
     - real
     - 0
     - White balance tint.
   * - ``filament.cg.slope``
     - real(3)
     - 1 1 1
     - ASC CDL slope.
   * - ``filament.cg.offset``
     - real(3)
     - 0 0 0
     - ASC CDL offset.
   * - ``filament.cg.power``
     - real(3)
     - 1 1 1
     - ASC CDL power.
   * - ``filament.cg.shadows``
     - real(4)
     - 1 1 1 1
     - Shadow color shift; the fourth component shifts luminance.
   * - ``filament.cg.midtones``
     - real(4)
     - 1 1 1 1
     - Midtone color shift; the fourth component shifts luminance.
   * - ``filament.cg.highlights``
     - real(4)
     - 1 1 1 1
     - Highlight color shift; the fourth component shifts luminance.
   * - ``filament.cg.tonal_ranges``
     - real(4)
     - 0 0.333 0.55 1
     - Boundaries of the shadow and highlight tonal ranges.
   * - ``filament.cg.shadow_gamma``
     - real(3)
     - 1 1 1
     - Gamma adjustment of shadows.
   * - ``filament.cg.mid_point``
     - real(3)
     - 1 1 1
     - Boundary between shadows and highlights.
   * - ``filament.cg.highlight_scale``
     - real(3)
     - 1 1 1
     - Scale of highlights.
   * - ``filament.cg.contrast``
     - real
     - 1
     - Contrast.
   * - ``filament.cg.vibrance``
     - real
     - 1
     - Vibrance.
   * - ``filament.cg.saturation``
     - real
     - 1
     - Saturation.
   * - ``filament.cg.tone_mapping``
     - string
     - pbr_neutral
     - Tone mapping operator: ``aces``, ``aces_legacy``, ``filmic``, ``linear`` or ``pbr_neutral``.
   * - ``filament.cg.luminance_scaling``
     - bool
     - 0
     - Scale luminance to desaturate very bright highlights.
   * - ``filament.cg.gamut_mapping``
     - bool
     - 0
     - Map out-of-gamut colors into the output gamut.

Exponential height fog:

.. list-table::
   :widths: 32 10 12 46
   :header-rows: 1

   * - Flag
     - Type
     - Default
     - Description
   * - ``filament.fog.enabled``
     - bool
     - --
     - Enable fog.
   * - ``filament.fog.color``
     - real(3)
     - --
     - Fog color.
   * - ``filament.fog.distance``
     - real
     - --
     - Distance from the camera at which the fog starts.
   * - ``filament.fog.density``
     - real
     - --
     - Fog density.
   * - ``filament.fog.cutOffDistance``
     - real
     - --
     - Distance beyond which fog is not applied.
   * - ``filament.fog.maximumOpacity``
     - real
     - --
     - Maximum opacity of the fog.
   * - ``filament.fog.height``
     - real
     - --
     - Height above which the fog density decreases.
   * - ``filament.fog.heightFalloff``
     - real
     - --
     - Falloff of fog density with height.
   * - ``filament.fog.inScatteringStart``
     - real
     - --
     - Distance at which light in-scattering starts.
   * - ``filament.fog.inScatteringSize``
     - real
     - --
     - Size of the light in-scattering halo.

Shadows and anti-aliasing:

.. list-table::
   :widths: 32 10 12 46
   :header-rows: 1

   * - Flag
     - Type
     - Default
     - Description
   * - ``filament.shadows.type``
     - int
     - 0
     - Shadow algorithm: 0: PCF, 1: VSM, 2: DPCF, 3: PCSS.
   * - ``filament.shadows.map_size``
     - int
     - 2048
     - Shadow map resolution, in texels.
   * - ``filament.msaa.enabled``
     - bool
     - 1
     - Enable multi-sample anti-aliasing.

Scene and fallback lighting:

.. list-table::
   :widths: 32 10 12 46
   :header-rows: 1

   * - Flag
     - Type
     - Default
     - Description
   * - ``filament.clearColor``
     - real(4)
     - 0 0 0 1
     - Background color.
   * - ``filament.fallback.environment_light_intensity``
     - real
     - 5000
     - Intensity of the fallback environment light.
   * - ``filament.fallback.scene_light_intensity``
     - real
     - 80000
     - Total intensity distributed among the model's lights.
   * - ``filament.fallback.head_light_intensity``
     - real
     - 0
     - Intensity of the headlight.

.. _fiArchitecture:

Architecture
~~~~~~~~~~~~

The renderer is organized in layers, from the bottom up:

**The mjrf API.** The public interface to the renderer is the ``mjrf`` C API, declared in `mjrfilament.h
<https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjrfilament.h>`__. Unlike the classic ``mjr`` API,
which redraws an :ref:`mjvScene` every frame, ``mjrf`` is *retained-mode* and *asynchronous*: applications create
long-lived objects -- contexts, scenes, meshes, textures, lights, renderables and render targets -- mutate them
incrementally, and submit batches of render and read-pixels requests. Requests are processed on a dedicated render
thread; :ref:`mjrf_render` returns a frame handle which can be waited on with :ref:`mjrf_waitForFrame`. The API itself
is not thread-safe: all calls are expected from a single thread. See the ``mjrf``
:ref:`type<tyFilamentRenderStructure>` and :ref:`function<FilamentRenderingApi>` reference.

**The model layer.** The support library in ``src/render/filament/support`` builds ``mjrf`` objects directly from
:ref:`mjModel` -- meshes, textures, materials, lights and the skybox -- and updates their poses and colors each
frame from :ref:`mjData`. In this path no ``mjvScene`` is involved: geometry is uploaded once and updated in place.

**The compatibility layer.** ``src/experimental/filament`` implements the classic ``mjr`` API on top of ``mjrf``, so
existing applications can switch renderers without code changes. A scene bridge reconciles the retained Filament scene
against the ``mjvScene`` passed to :ref:`mjr_render` on every call, so the standard
:ref:`mjv_updateScene`/:ref:`mjr_render` loop -- including decorative geoms and perturbations -- works unchanged.
Functions not supported by this layer call :ref:`mju_error` when invoked: auxiliary buffers and the 2d drawing
functions (:ref:`mjr_text`, :ref:`mjr_rectangle`, :ref:`mjr_figure`, ...); :ref:`mjr_readPixels` is supported for
offscreen rendering only.

In this design, ``mjvScene`` remains the interface for abstract visualization -- visualization options, decorative
geoms and user-defined geoms -- while bulk model geometry flows directly from ``mjModel`` and ``mjData`` to the GPU.

**The platform layer.** ``src/experimental/platform`` contains the application toolkit used by Studio: ``hal`` for
windowing and graphics backend selection, ``ux`` for the GUI framework, and ``sim`` for the simulation loop: stepping
control, state history and profiling.

The code is located as follows; directories under ``experimental`` will move as the code matures:

.. list-table::
   :widths: 36 64
   :header-rows: 1

   * - Location
     - Contents
   * - ``include/mujoco/mjrfilament.h``
     - Public ``mjrf`` API.
   * - ``src/render/filament``
     - The core renderer: ``mjrf`` implementation, model layer, materials and shaders.
   * - ``src/experimental/filament``
     - The ``mjr`` compatibility layer and the ``mjvScene`` bridge.
   * - ``src/experimental/platform``
     - Application toolkit: windowing (``hal``), GUI framework (``ux``), simulation loop (``sim``).
   * - ``src/experimental/studio``
     - The MuJoCo Studio application, including its web build.

GUI rendering
^^^^^^^^^^^^^

The GUI framework is built on `Dear ImGui <https://github.com/ocornut/imgui>`__. ImGui provides the immediate-mode
widget toolkit, while rendering is done by Filament itself: each frame, the ImGui draw lists are converted into
``mjrf`` renderables and drawn in the same pass as the scene. A single rendering pipeline therefore serves both
simulation and interface on every backend, including the browser.

.. _Studio:

MuJoCo Studio
~~~~~~~~~~~~~

MuJoCo Studio is the next iteration of the :ref:`simulate<saSimulate>` application, built on the Filament renderer and
the Dear ImGui-based GUI framework. Studio loads models in XML, MJB and MJZ format -- from the command line, the file
dialog, or by drag-and-drop -- and ports the simulate interface. New capabilities include an integrated profiler,
in-app editing of the model spec, light and dark themes, and persistent user settings.

Build and run Studio from the top-level directory:

.. code-block:: shell

   bash src/experimental/studio/build.sh
   ./build/bin/mujoco_studio [model file]

The ``--gfx`` flag selects the graphics backend: ``opengl`` or ``vulkan``, each with ``_headless`` and ``_software``
variants; the default is platform-dependent. On Linux, Studio requires X11; Wayland is not yet supported.

Python bindings for Studio, including a ``launch_passive``-style API, are under development.

.. _Live:

MuJoCo Live
~~~~~~~~~~~

`MuJoCo Live <https://live.mujoco.org>`__ is Studio compiled to WebAssembly, running entirely in the browser. Models
are loaded by dragging them onto the page, or with the ``model`` URL parameter, which accepts both plain URLs and the
``github:`` shorthand for files hosted on GitHub. For example, this link loads a model from the `MuJoCo Menagerie
<https://github.com/google-deepmind/mujoco_menagerie>`__:

   `live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/unitree_go2/scene.xml
   <https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/unitree_go2/scene.xml>`__

Assets referenced by the model are resolved relative to the model's URL and prefetched in parallel before compilation.

Live is a static site, rebuilt by continuous integration from the ``live`` branch of the MuJoCo repository, which is
updated on every MuJoCo release.
