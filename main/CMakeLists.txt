# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS main.c         # List of source files
         wifi_app.c
         http_server.c
    INCLUDE_DIRS "."    # Public include directories
    PRIV_INCLUDE_DIRS   # Private include directories (if needed)
    EMBED_FILES webpage/app.css 
                webpage/app.js 
                webpage/favicon.ico 
                webpage/index.html 
                webpage/jquery-3.3.1.min.js
    REQUIRES            # Public dependencies (if needed)
    PRIV_REQUIRES       # Private dependencies (if needed)
)