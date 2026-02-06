package com.ntu.group24.android.models;

import androidx.lifecycle.LiveData;
import androidx.lifecycle.MutableLiveData;
import androidx.lifecycle.ViewModel;

public class RobotViewModel extends ViewModel {
    private final MutableLiveData<String> moveRequest = new MutableLiveData<>();
    private final MutableLiveData<String> incomingCommand = new MutableLiveData<>();
    public void requestMovement(String direction) {
        moveRequest.setValue(direction);
    }

    public LiveData<String> getMoveRequest() {
        return moveRequest;
    }

    public void setIncomingCommand(String cmd) {
        incomingCommand.setValue(cmd);
    }

    public LiveData<String> getIncomingCommand() {
        return incomingCommand;
    }
}