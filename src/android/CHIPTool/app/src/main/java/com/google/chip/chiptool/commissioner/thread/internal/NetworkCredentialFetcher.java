package com.google.chip.chiptool.commissioner.thread.internal;

import android.util.Log;
import androidx.annotation.NonNull;
import com.google.chip.chiptool.commissioner.thread.BorderAgentInfo;
import com.google.chip.chiptool.commissioner.thread.CommissionerUtils;
import com.google.chip.chiptool.commissioner.thread.ThreadCommissionerException;
import com.google.chip.chiptool.commissioner.thread.ThreadNetworkCredential;
import io.openthread.commissioner.ActiveOperationalDataset;
import io.openthread.commissioner.ByteArray;
import io.openthread.commissioner.ChannelMask;
import io.openthread.commissioner.Commissioner;
import io.openthread.commissioner.CommissionerHandler;
import io.openthread.commissioner.Config;
import io.openthread.commissioner.Error;
import io.openthread.commissioner.LogLevel;
import io.openthread.commissioner.Logger;
import java.net.InetAddress;

class NetworkCredentialFetcher {

  private static final String TAG = NetworkCredentialFetcher.class.getSimpleName();

  public ThreadNetworkCredential fetchNetworkCredential(
      @NonNull BorderAgentInfo borderAgentInfo, @NonNull byte[] pskc)
      throws ThreadCommissionerException {
    ActiveOperationalDataset activeOperationalDataset =
        fetchNetworkCredential(borderAgentInfo.host, borderAgentInfo.port, pskc);
    return new ThreadNetworkCredential(
        CommissionerUtils.getByteArray(activeOperationalDataset.getRawTlvs()));
  }

  private ActiveOperationalDataset fetchNetworkCredential(
      @NonNull InetAddress address, int port, @NonNull byte[] pskc)
      throws ThreadCommissionerException {
    Commissioner nativeCommissioner = Commissioner.create(new NativeCommissionerHandler());

    Config config = new Config();
    config.setId("TestComm");
    config.setDomainName("TestDomain");
    config.setEnableCcm(false);
    config.setPSKc(CommissionerUtils.getByteArray(pskc));
    config.setLogger(new NativeCommissionerLogger());

    // Initialize the native commissioner
    throwIfFail(nativeCommissioner.init(config));

    // Petition to be the active commissioner in the Thread Network.
    String[] existingCommissionerId = new String[1];
    throwIfFail(
        nativeCommissioner.petition(existingCommissionerId, address.getHostAddress(), port));

    // Fetch Active Operational Dataset.
    ActiveOperationalDataset activeOperationalDataset = new ActiveOperationalDataset();
    throwIfFail(nativeCommissioner.getActiveDataset(activeOperationalDataset, 0xFFFF));
    nativeCommissioner.resign();

    return activeOperationalDataset;
  }

  private void throwIfFail(Error error) throws ThreadCommissionerException {
    throw new ThreadCommissionerException(error.getCode().swigValue(), error.getMessage());
  }
}

class NativeCommissionerLogger extends Logger {
  private static final String TAG = "NativeCommissioner";

  @Override
  public void log(LogLevel level, String region, String msg) {
    Log.d(TAG, String.format("[ %s ]: %s", region, msg));
  }
}

class NativeCommissionerHandler extends CommissionerHandler {
  private static final String TAG = NativeCommissionerHandler.class.getSimpleName();

  @Override
  public String onJoinerRequest(ByteArray joinerId) {
    Log.d(TAG, "A joiner is requesting commissioning");
    return "";
  }

  @Override
  public void onJoinerConnected(ByteArray joinerId, Error error) {
    Log.d(TAG, "A joiner is connected");
  }

  @Override
  public boolean onJoinerFinalize(
      ByteArray joinerId,
      String vendorName,
      String vendorModel,
      String vendorSwVersion,
      ByteArray vendorStackVersion,
      String provisioningUrl,
      ByteArray vendorData) {
    Log.d(TAG, "A joiner is finalizing");
    return true;
  }

  @Override
  public void onKeepAliveResponse(Error error) {
    Log.d(TAG, "received keep-alive response: " + error.toString());
  }

  @Override
  public void onPanIdConflict(String peerAddr, ChannelMask channelMask, int panId) {
    Log.d(TAG, "received PAN ID CONFLICT report");
  }

  @Override
  public void onEnergyReport(String aPeerAddr, ChannelMask aChannelMask, ByteArray aEnergyList) {
    Log.d(TAG, "received ENERGY SCAN report");
  }

  @Override
  public void onDatasetChanged() {
    Log.d(TAG, "Thread Network Dataset chanaged");
  }
}

/*
class CommissionerWorker extends Worker {

  private static final String TAG = CommissionerWorker.class.getSimpleName();

  private CHIPDeviceInfo deviceInfo;
  private ThreadNetworkInfo threadNetworkInfo;
  private byte[] pskc;

  private static Commissioner nativeCommissioner;

  public CommissionerWorker(@NonNull Context context, @NonNull WorkerParameters workerParams) {
    super(context, workerParams);

    deviceInfo =
        new Gson()
            .fromJson(getInputData().getString(Constants.KEY_DEVICE_INFO), CHIPDeviceInfo.class);
    threadNetworkInfo =
        new Gson()
            .fromJson(getInputData().getString(Constants.KEY_NETWORK_INFO), ThreadNetworkInfo.class);
    pskc = new Gson().fromJson(getInputData().getString(Constants.KEY_PSKC), byte[].class);

    nativeCommissioner = Commissioner.create(new NativeCommissionerHandler(this));

    Config config = new Config();
    config.setId("TestComm");
    config.setDomainName("TestDomain");
    config.setEnableCcm(false);
    config.setPSKc(CommissionerUtils.getByteArray(pskc));
    config.setLogger(new NativeCommissionerLogger());

    nativeCommissioner.init(config);
  }

  String getPskd() {
    return String.format("%09u", deviceInfo.getSetupPinCode());
  }

  @NonNull
  @Override
  public Result doWork() {
    if (nativeCommissioner != null) {
      nativeCommissioner.resign();
    }

    setProgressAsync(StateToData("petitioning..."));

    String[] existingCommissionerId = new String[1];
    Error error =
        nativeCommissioner.petition(
            existingCommissionerId, threadNetworkInfo.getHost().getHostAddress(), threadNetworkInfo.getPort());
    if (error.getCode() != ErrorCode.kNone) {
      return errorToResult(error);
    }

    // Store the PSKc after successfully connecting to the current Border Agent.
    ThreadNetworkCredential networkCredential = new ThreadNetworkCredential(threadNetworkInfo.getNetworkName(), threadNetworkInfo
        .getExtendedPanId(), pskc, null);
    try {
      NetworkCredentialDatabase.getDatabase(getApplicationContext()).insertNetworkCredential(networkCredential);
    } catch (ExecutionException e) {
      e.printStackTrace();
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    setProgressAsync(StateToData("commissioner connected"));

    // TODO(wgtdkp): get active operational dataset
    //ActiveOperationalDataset
    //nativeCommissioner.getActiveDataset();

    nativeCommissioner.resign();

    return errorToResult(new Error(ErrorCode.kNone, ""));
  }

  @Override
  public void onStopped() {
    if (nativeCommissioner != null) {
      nativeCommissioner.resign();
    }
  }

  private Result errorToResult(Error error) {
    Data.Builder dataBuilder = new Data.Builder();

    if (error.getCode() == ErrorCode.kNone) {
      dataBuilder.putString(Constants.KEY_COMMISSIONING_STATUS, "commission device success!");
      dataBuilder.putBoolean(Constants.KEY_SUCCESS, true);
      return Result.success(dataBuilder.build());
    } else {
      dataBuilder.putString(Constants.KEY_COMMISSIONING_STATUS, error.toString());
      dataBuilder.putBoolean(Constants.KEY_SUCCESS, false);
      return Result.failure(StateToData(error.toString()));
    }
  }

  private Data StateToData(String state) {
    return new Data.Builder().putString(Constants.KEY_COMMISSIONING_STATUS, state).build();
  }
}
*/
