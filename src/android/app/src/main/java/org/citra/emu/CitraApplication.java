// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

package org.citra.emu;

import android.app.Application;
import org.citra.emu.utils.DirectoryInitialization;
import org.citra.emu.utils.PermissionsHandler;

public class CitraApplication extends Application {
    static {
        System.loadLibrary("main");
    }

    @Override
    public void onCreate() {
        super.onCreate();

        if (PermissionsHandler.hasWriteAccess(getApplicationContext()))
            DirectoryInitialization.start(getApplicationContext());
    }
}
