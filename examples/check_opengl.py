import sys

try:
    import OpenGL
    from OpenGL.GL import glGetString, GL_VERSION, GL_SHADING_LANGUAGE_VERSION, GL_VENDOR, GL_RENDERER
    import glfw
except ImportError as e:
    print(f"Error importing dependencies: {e}")
    print("Please install them using: pip install PyOpenGL glfw")
    sys.exit(1)

def main():
    # Initialize GLFW
    if not glfw.init():
        print("Failed to initialize GLFW")
        sys.exit(1)

    # Create a hidden window to initialize an OpenGL context
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    window = glfw.create_window(100, 100, "OpenGL Version Check", None, None)
    if not window:
        glfw.terminate()
        print("Failed to create GLFW window context")
        sys.exit(1)

    # Make the window's context current
    glfw.make_context_current(window)

    # Query OpenGL information
    version = glGetString(GL_VERSION).decode('utf-8')
    vendor = glGetString(GL_VENDOR).decode('utf-8')
    renderer = glGetString(GL_RENDERER).decode('utf-8')

    try:
        glsl_version = glGetString(GL_SHADING_LANGUAGE_VERSION).decode('utf-8')
    except Exception:
        glsl_version = "Unknown / Not Supported"

    print("=" * 50)
    print("OpenGL System Information")
    print("=" * 50)
    print(f"OpenGL Version:   {version}")
    print(f"GLSL Version:     {glsl_version}")
    print(f"Vendor:           {vendor}")
    print(f"Renderer:         {renderer}")
    print("=" * 50)

    # Clean up
    glfw.destroy_window(window)
    glfw.terminate()

if __name__ == "__main__":
    main()
