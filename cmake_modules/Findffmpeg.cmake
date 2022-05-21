# FFMPEG cmake project-config input for ./configure scripts
set(libdir "/usr/lib/x86_64-linux-gnu")
set(FFMPEG_LIBDIR "/usr/lib/x86_64-linux-gnu")
set(FFMPEG_INCLUDE_DIRS "/usr/include/x86_64-linux-gnu")
set(FFMPEG_FOUND 1)
set(FFMPEG_LIBRARIES "-L${FFMPEG_LIBDIR} -lavcodec -lavformat -lpostproc -lavdevice -lavresample -lswresample -lavfilter -lswscale -lavutil")
string(STRIP "${FFMPEG_LIBRARIES}" FFMPEG_LIBRARIES)