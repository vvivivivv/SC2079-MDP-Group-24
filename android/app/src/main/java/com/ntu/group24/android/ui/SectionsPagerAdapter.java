package com.ntu.group24.android.ui;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentActivity;
import androidx.viewpager2.adapter.FragmentStateAdapter;

public class SectionsPagerAdapter extends FragmentStateAdapter {

    public SectionsPagerAdapter(@NonNull FragmentActivity fragmentActivity) {
        super(fragmentActivity);
    }

    @NonNull
    @Override
    public Fragment createFragment(int position) {
        switch (position) {
            case 0: return new MapFragment();
            case 1: return new ControlFragment();
            case 2: return new CommunicationsFragment();
            case 3: return new BluetoothFragment();
            default: return new MapFragment();
        }
    }

    @Override
    public int getItemCount() {
        return 4;
    }
}
