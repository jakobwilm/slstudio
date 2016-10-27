#include "Projector.h"

#ifdef WITH_PROJECTOROPENGL
#include "OpenGLContext.h"
#include "ProjectorOpenGL.h"
#endif

#ifdef WITH_LC3000API
#include "ProjectorLC3000.h"
#endif

#ifdef WITH_LC4500API
#include "ProjectorLC4500.h"
#endif

#ifdef WITH_PROJECTORQT
#include "ProjectorQtGL.h"
#endif

Projector* Projector::NewProjector(ProjectorType projector, unsigned int screenNum){
    switch(projector){
#ifdef WITH_PROJECTOROPENGL
    case projectorTypeOpenGL:
        return new ProjectorOpenGL(screenNum);
#endif
#ifdef WITH_LC3000API
    case projectorTypeLC3000:
        return new ProjectorLC3000(0);
#endif
#ifdef WITH_LC4500API
    case projectorTypeLC4500:
        return new ProjectorLC4500(0);
#endif
#ifdef WITH_PROJECTORQT
    case projectorTypeQtGL:
        return new ProjectorQtGL();
#endif
    default:
        return 0;
    }
}

std::set<ProjectorType> Projector::GetProjectorList(){
    std::set<ProjectorType> projectorList;

#ifdef WITH_PROJECTOROPENGL
    projectorList.insert(projectorTypeOpenGL);
#endif

#ifdef WITH_LC3000API
    projectorList.insert(projectorTypeLC3000);
#endif

#ifdef WITH_LC4500API
    projectorList.insert(projectorTypeLC4500);
#endif

#ifdef WITH_PROJECTORQT
    projectorList.insert(projectorTypeQtGL);
#endif

    return projectorList;
}

std::vector<ScreenInfo> Projector::GetScreenInfo(){
#ifdef WITH_PROJECTOROPENGL
    return OpenGLContext::GetScreenInfo();
#else
    return std::vector<ScreenInfo>();
#endif
}
