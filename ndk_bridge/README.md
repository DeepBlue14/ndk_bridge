
- Create a project root directory [your_proj_root_dir]
- Place this directory (e.g. sample_app) in that directory.
- Download the roscpp_android_ndk binary, and unzip in at [your_proj_root_dir]
- Open sample_app in Android Studio (note: initially, the C++ part of the project will **not** appear in the file tree)
- right-click the root of the file tree (e.g. "app"), and select "Link C++ Project with Gradle"
    - for the Build System option, select ndk-build
    - for Project Path, select /[your_path]/sample_app/app/src/main/jni/Android.mk
      Once this is done, you should see "sample_app (Shared Library)" as well as the C++ files added to the file tree.
      
      
- http://stackoverflow.com/questions/23146677/changing-the-style-of-braces-in-android-studio
- http://tools.android.com/tech-docs/new-build-system/gradle-experimental/migrate-to-stable
