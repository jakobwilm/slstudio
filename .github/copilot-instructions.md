# SLStudio engineering principles

- Follow **YAGNI** principles: build for demonstrated needs, not hypothetical features, configuration, or frameworks.
- Ask before adding new tests. Generally favor few, but valuable tests.
- Don't add new/additional text to Readme.md unless asked to. Small corrections are ok.
- Do not consider backward compatibility with older versions of setting files/sequences/etc this software.
- Favor simple, readable, cohesive code. Refactor incrementally without unrelated redesigns.
- Reuse maintained libraries and helpers; keep shared metadata in one place.
- Make ownership and lifecycle transitions explicit; centralize cancellation and cleanup.
- During camera handovers, finish SDK calls and release the old device off the UI thread before opening its replacement.
- Keep scan and camera lifetimes separate: stopping a scan preserves preview and completed sequences; stopping the camera preserves configuration.
- Separate algorithms, acquisition, presentation, and tests. Keep OpenGL GUI-owned and non-rendering logic headlessly testable.
- Preserve defaults, identifiers, ordering, and saved-data compatibility. Wire options through APIs, GUI, pipelines, and persistence.
- Test observable behavior, failure paths, and persistence; use GUI smoke tests for integration. Omit smoke code with `BUILD_TESTING=OFF` and distinguish simulation from hardware validation.
- The [README](../README.md) should contain an overview of scope, usage, installation. The code itself documents engineering choices and implementation details.
