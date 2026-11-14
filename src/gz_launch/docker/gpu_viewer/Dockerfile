ARG BASE_IMAGE
FROM ${BASE_IMAGE}
ARG VIRTUALGL_VERSION=3.1.5
RUN curl -fL --connect-timeout 15 --max-time 180 \
      -o /tmp/virtualgl.deb \
      https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_amd64.deb \
    && echo 'df3f7788ce41b182a47c0d298e5cd6d2d63579522cb41825970b7726e825485e  /tmp/virtualgl.deb' | sha256sum -c - \
    && dpkg -i /tmp/virtualgl.deb \
    && rm /tmp/virtualgl.deb
RUN printf '%s\n' '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' > /opt/VirtualGL/nvidia_egl.json
ENV NVIDIA_DRIVER_CAPABILITIES=all \
    LIBGL_ALWAYS_SOFTWARE=0 LIBGL_ALWAYS_INDIRECT=0 \
    __GLX_VENDOR_LIBRARY_NAME=nvidia \
    __EGL_VENDOR_LIBRARY_FILENAMES=/opt/VirtualGL/nvidia_egl.json \
    VGL_FPS=30
LABEL c43.viewer="virtualgl-3.1.5-egl-proxy"
ENTRYPOINT []
CMD ["sleep", "infinity"]
