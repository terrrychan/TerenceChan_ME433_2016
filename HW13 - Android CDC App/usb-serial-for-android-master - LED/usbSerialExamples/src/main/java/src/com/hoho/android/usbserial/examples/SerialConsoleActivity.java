/* Copyright 2011-2013 Google Inc.
 * Copyright 2013 mike wakerly <opensource@hoho.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 *
 * Project home page: https://github.com/mik3y/usb-serial-for-android
 */

package src.com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.util.HexDump;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;


import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;

/**
 * Monitors a single {@link UsbSerialPort} instance, showing all data
 * received.
 *
 * @author mike wakerly (opensource@hoho.com)
 */

public class SerialConsoleActivity extends Activity implements TextureView.SurfaceTextureListener  {

    private final String TAG = SerialConsoleActivity.class.getSimpleName();

    /**
     * Driver instance, passed in statically via
     * {@link #show(Context, UsbSerialPort)}.
     *
     * <p/>
     * This is a devious hack; it'd be cleaner to re-create the driver using
     * arguments passed in with the {@link #startActivity(Intent)} intent. We
     * can get away with it because both activities will run in the same
     * process, and this is a simple demo.
     */

    private static UsbSerialPort sPort = null;

    private TextView mTitleTextView;
//    private TextView mDumpTextView;
//    private ScrollView mScrollView;
//    private CheckBox chkDTR;
//    private CheckBox chkRTS;

    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640,480,Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;
    static long prevtime = 0; // for FPS calculation

    SeekBar myControl;
    TextView myTextView;
//    SeekBar myControl2;
//    TextView myTextView2;

    int progressChanged1 = 0;
    int thresholdValue1 = 0;
    int progressChanged2 = 0;
    int thresholdValue2 = 0;
    int[] thresholdedColors1 = new int[bmp.getWidth()];
    int[] thresholdedColors2 = new int[bmp.getWidth()];
    int average_red1, average_red2, average_green1, average_green2, average_blue1, average_blue2;
    int sendPWM;
    float COM_difference;
    float COM_average;
    String sendString;

    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();

    private SerialInputOutputManager mSerialIoManager;

    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {

        @Override
        public void onRunError(Exception e) {
            Log.d(TAG, "Runner stopped.");
        }

        @Override
        public void onNewData(final byte[] data) {
//        public void onNewData(final int[] data) {
            SerialConsoleActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
//                    sendArray[4] = (byte) ('\n');
                    SerialConsoleActivity.this.updateReceivedData(data);

                }
            });
        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.serial_console);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        mTitleTextView = (TextView) findViewById(R.id.demoTitle);

//        mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
//        mSurfaceHolder = mSurfaceView.getHolder();
//
//        mTextureView = (TextureView) findViewById(R.id.textureview);
//        mTextureView.setSurfaceTextureListener(this);

        mTextView = (TextView) findViewById(R.id.cameraStatus);

        paint1.setColor(0xffff0000); // red
        paint1.setTextSize(24);

        // instantiate the seekbar and textview
        myControl = (SeekBar) findViewById(R.id.seek1);
        myTextView = (TextView) findViewById(R.id.textView01);
        myTextView.setText("Enter whatever you Like!");
        setMyControlListener();

        // instantiate the seekbar and textview
//        myControl2 = (SeekBar) findViewById(R.id.seek2);
//        myTextView2 = (TextView) findViewById(R.id.textView02);
//        myTextView2.setText("Enter whatever you Like!");
//        setMyControlListener2();
    }

    private void setMyControlListener() {
        myControl.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged1 = progress;
//                thresholdValue1 = progressChanged1*10;
                thresholdValue1 = progressChanged1*255/100;
//                myTextView.setText("The value is: "+progress+ "."+ "The threshold value is: " +thresholdValue1);

                String sendString = String.valueOf(progressChanged1) + String.valueOf('\n');
                String doc = "The value sent by the PIC is = " + String.valueOf(progressChanged1);
//        String doc = new String(data, "ISO-8859-1");

                try {
                    sPort.write(sendString.getBytes(),10); // 10 is the timeout (error)
                }
                catch (IOException e) {}


//        try { sPort.write(sData, 10); } catch (IOException e) { } // know for sure that its sending data to the PIC!

                myTextView.setText(doc);

            }
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

//    private void setMyControlListener2() {
//        myControl2.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
//
//            @Override
//            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
//                progressChanged2 = progress;
////                thresholdValue2 = progressChanged2*10;
//                thresholdValue2 = progressChanged2*255/100;
////                myTextView2.setText("The value is: "+progress+ "."+ "The threshold value is: " +thresholdValue2);
//            }
//            @Override
//            public void onStartTrackingTouch(SeekBar seekBar) {
//            }
//            @Override
//            public void onStopTrackingTouch(SeekBar seekBar) {
//
//            }
//        });
//    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setColorEffect(Camera.Parameters.EFFECT_NONE); // black and white
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        parameters.setWhiteBalance(Camera.Parameters.WHITE_BALANCE_DAYLIGHT);
        parameters.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
//        parameters.setSceneMode(Camera.Parameters.SCENE_MODE_STEADYPHOTO);
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // Invoked every time there's a new Camera preview frame
        mTextureView.getBitmap(bmp);


        final Canvas c = mSurfaceHolder.lockCanvas();
        if (c != null) {

            int aCOM1 = 0;
            int aCOM2 = 0;
            int pixels_search = 50;

            int[] pixels1 = new int[bmp.getWidth()];
            int[] pixels2 = new int[bmp.getWidth()];
            // 100 - 300 the best so far?
            int startY1 = 330; // which row in the bitmap to analyse to read
            int startY2 = 400; // which row in the bitmap to analyse to read
            int init_startY1 = startY1;
            int init_startY2 = startY2;
            int average_startY1 = init_startY1 + pixels_search/2;
            int average_startY2 = init_startY2 + pixels_search/2;
            int COM1 = 0;
            int COM2 = 0;


            for (int j = 0; j < pixels_search; j++) {
                startY1 = startY1 + 1;
                startY2 = startY2 + 1;

                // only look at one row in the image
                bmp.getPixels(pixels1, 0, bmp.getWidth(), 0, startY1, bmp.getWidth(), 1); // (array name, offset inside array, stride (size of row), start x, start y, num pixels to read per row, num rows to read)
                bmp.getPixels(pixels2, 0, bmp.getWidth(), 0, startY2, bmp.getWidth(), 1); // (array name, offset inside array, stride (size of row), start x, start y, num pixels to read per row, num rows to read)

                // pixels[] is the RGBA data (in black an white).
                // instead of doing center of mass on it, decide if each pixel is dark enough to consider black or white
                // then do a center of mass on the thresholded array
                int[] thresholdedPixels1 = new int[bmp.getWidth()];
                int[] thresholdedPixels2 = new int[bmp.getWidth()];
                int wbTotal1 = 0; // total mass
                int wbTotal2 = 0; // total mass 2
                int wbCOM1 = 0; // total (mass time position)
                int wbCOM2 = 0; // total (mass time position)
//                int COM1 = 0;
//                int COM2 = 0;

                for (int i = 0; i < bmp.getWidth(); i++) {
                    // sum the red, green and blue, subtract from 255 to get the darkness of the pixel.
                    // if it is greater than some value (600 here), consider it black
                    // play with the 600 value if you are having issues reliably seeing the line

                    if (255 - (red(pixels1[i])) > thresholdValue1) {
//                    if (255 * 3 - (red(pixels1[i]) + green(pixels1[i]) + blue(pixels1[i])) > thresholdValue1) {
//                if (255*3-(red(pixels1[i])+green(pixels1[i])+blue(pixels1[i])) > 600) {
                        thresholdedPixels1[i] = 0;
//                        thresholdedPixels1[i] = 255 * 3;
                        thresholdedColors1[i] = rgb(0, 0, 0);
                    } else {
//                        thresholdedPixels1[i] = 0;
                        thresholdedPixels1[i] = 255 * 3;
                        thresholdedColors1[i] = rgb(0, 255, 0);
                    }

                    wbTotal1 = wbTotal1 + thresholdedPixels1[i];
                    wbCOM1 = wbCOM1 + thresholdedPixels1[i] * i;

                    if (i == average_startY1){
                        average_red1 = red(pixels1[i]);
                        average_green1 = green(pixels1[i]);
                        average_blue1 = blue(pixels1[i]);
                    }
                }

                for (int i = 0; i < bmp.getWidth(); i++) {
                    // sum the red, green and blue, subtract from 255 to get the darkness of the pixel.
                    // if it is greater than some value (600 here), consider it black
                    // play with the 600 value if you are having issues reliably seeing the line
//                    if (255 * 3 - (red(pixels2[i]) + green(pixels2[i]) + blue(pixels2[i])) > thresholdValue2) {
                    if (255 - (red(pixels2[i])) > thresholdValue2) {
//                if (255*3-(red(pixels2[i])+green(pixels2[i])+blue(pixels2[i])) > 600) {
                        thresholdedPixels2[i] = 0;
//                        thresholdedPixels2[i] = 255 * 3;
                        thresholdedColors2[i] = rgb(0, 0, 0);
                    } else {
//                        thresholdedPixels2[i] = 0;
                        thresholdedPixels2[i] = 255 * 3;
                        thresholdedColors2[i] = rgb(0, 255, 0);
                    }
                    wbTotal2 = wbTotal2 + thresholdedPixels2[i];
                    wbCOM2 = wbCOM2 + thresholdedPixels2[i] * i;

                    if (i == average_startY2){
                        average_red2 = red(pixels2[i]);
                        average_green2 = green(pixels2[i]);
                        average_blue2 = blue(pixels2[i]);
                    }
                }

                bmp.setPixels(thresholdedColors1, 0, bmp.getWidth(), 0, startY1, bmp.getWidth(), 1);
                bmp.setPixels(thresholdedColors2, 0, bmp.getWidth(), 0, startY2, bmp.getWidth(), 1);

                //watch out for divide by 0
                if (wbTotal1 <= 0) {
                    COM1 = bmp.getWidth() / 2; // set it to the center
                } else {
                    COM1 = wbCOM1 / wbTotal1;
                }

                //watch out for divide by 0
                if (wbTotal2 <= 0) {
                    COM2 = bmp.getWidth() / 2;
                } else {
                    COM2 = wbCOM2 / wbTotal2;
                }

                aCOM1 = aCOM1 + COM1;
                aCOM2 = aCOM2 + COM2;


                // draw a circle where you think the COM is
//                canvas.drawCircle(COM1, startY1, 5, paint1);
//                canvas.drawCircle(COM2, startY2, 5, paint1);
            }

            aCOM1 = aCOM1 / (pixels_search);
            aCOM2 = aCOM2 / (pixels_search);

            canvas.drawCircle(aCOM1, average_startY1, 5, paint1);
            canvas.drawCircle(aCOM2, average_startY2, 5, paint1);

            // COM_difference will be between -319 and +319.... has to map a range of 100
            COM_difference = aCOM1- aCOM2;
            COM_average = (aCOM1 + aCOM2)/2;
            sendPWM = (int) (COM_difference * 100 / 320.0);

//            String sendString = String.valueOf(COM_average) + String.valueOf('\n');


            // also write the value as text
            // max COM = 600

            canvas.drawText("R =" + average_red1 + " " +
                            "G =" + average_green1 + " " +
                            "B =" + average_blue1, 10, 100, paint1);
            canvas.drawText("aCOM1 = " + aCOM1, 10, 125, paint1);

            canvas.drawText("R =" + average_red2 + " " +
                    "G =" + average_green2 + " " +
                    "B =" + average_blue2, 10, 150, paint1);
            int actual_command;
            if (COM_average < 320){
                actual_command = (int) (6000.0 * ((float) COM_average) / 320.0);
            }
            else if (COM_average > 320) {
                actual_command = (int) (6000.0 * ((float) (640.0 - COM_average)) / 320.0);
            }
            else {
                actual_command = 6000;
            }

            canvas.drawText("aCOM2 = " + aCOM2 + "COM_avg = " + COM_average +  "OCRS = " + actual_command,10, 175, paint1);

//            canvas.drawText("COM2 = " + COM2, 10, 200, paint1);
            c.drawBitmap(bmp, 0, 0, null);
            mSurfaceHolder.unlockCanvasAndPost(c);

            // calculate the FPS to see how fast the code is running
            long nowtime = System.currentTimeMillis();
            long diff = nowtime - prevtime;
            mTextView.setText("FPS " + 1000/diff);
            prevtime = nowtime;

//
            String sendString = String.valueOf(progressChanged1) + String.valueOf('\n');
//
//            try {
//                sPort.write(sendString.getBytes(),10); // 10 is the timeout (error)
//            }
//            catch (IOException e) {}

//            try {
//                sPort.write(sendString.getBytes(),10); // 10 is the timeout (error)
//            }
//            catch (IOException e) {}

        }
    }


//        Don't need this since not listening to any commands?
//        chkDTR.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
//            @Override
//            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
//                try {
//                    sPort.setDTR(isChecked);
//                }catch (IOException x){}
//            }
//        });
//
//        chkRTS.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
//            @Override
//            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
//                try {
//                    sPort.setRTS(isChecked);
//                }catch (IOException x){}
//            }
//        });




    @Override
    protected void onPause() {
        super.onPause();
        stopIoManager();
        if (sPort != null) {
            try {
                sPort.close();
            } catch (IOException e) {
                // Ignore.
            }
            sPort = null;
        }
        finish();
    }

    void showStatus(TextView theTextView, String theLabel, boolean theValue){
        String msg = theLabel + ": " + (theValue ? "enabled" : "disabled") + "\n";
        theTextView.append(msg);
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.d(TAG, "Resumed, port=" + sPort);
        if (sPort == null) {
            mTitleTextView.setText("No serial device.");
        } else {
            final UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);

            UsbDeviceConnection connection = usbManager.openDevice(sPort.getDriver().getDevice());
            if (connection == null) {
                mTitleTextView.setText("Opening device failed");
                return;
            }

            try {
                sPort.open(connection);
                sPort.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

//                showStatus(mDumpTextView, "CD  - Carrier Detect", sPort.getCD());
//                showStatus(mDumpTextView, "CTS - Clear To Send", sPort.getCTS());
//                showStatus(mDumpTextView, "DSR - Data Set Ready", sPort.getDSR());
//                showStatus(mDumpTextView, "DTR - Data Terminal Ready", sPort.getDTR());
//                showStatus(mDumpTextView, "DSR - Data Set Ready", sPort.getDSR());
//                showStatus(mDumpTextView, "RI  - Ring Indicator", sPort.getRI());
//                showStatus(mDumpTextView, "RTS - Request To Send", sPort.getRTS());

            } catch (IOException e) {
                Log.e(TAG, "Error setting up device: " + e.getMessage(), e);
                mTitleTextView.setText("Error opening device: " + e.getMessage());
                try {
                    sPort.close();
                } catch (IOException e2) {
                    // Ignore.
                }
                sPort = null;
                return;
            }
            mTitleTextView.setText("Serial device: " + sPort.getClass().getSimpleName());
        }
        onDeviceStateChange();
    }

    private void stopIoManager() {
        if (mSerialIoManager != null) {
            Log.i(TAG, "Stopping io manager ..");
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if (sPort != null) {
            Log.i(TAG, "Starting io manager ..");
            mSerialIoManager = new SerialInputOutputManager(sPort, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    private void onDeviceStateChange() {
        stopIoManager();
        startIoManager();
    }

    // not receiving data so don't need this?
    private void updateReceivedData(byte[] data) {
        // Read 4 bytes:
        // 0x00000000 00 00 00 00
        final String message = "Read " + data.length + " bytes: \n"
                + HexDump.dumpHexString(data) + "\n\n";
        // don't need to display any of this!
//        mDumpTextView.append(message); // gets "read 4 0x0000 000 ..._"
//        mScrollView.smoothScrollTo(0, mDumpTextView.getBottom());

        // communicating!
//        byte[] sData = {'a',0};
        byte[] sData = {'a','e'};
//        byte[] sData = {'a'}; // don't think the second position is actually sending..

//        int i = 1;
        int j = 2; // the two numbers to send
        char i = 'e';
        int test = 2;
        int test2 = 3;
//        String sendString = String.valueOf(j) + " " + String.valueOf(i);
//        String sendString = String.valueOf(test) + String.valueOf(test2) + String.valueOf(i);
//        String sendString = String.valueOf(sendPWM)+ ('\n');


    }

    /**
     * Starts the activity, using the supplied driver instance.
     *
     * @param context
     * @param driver
     */
    static void show(Context context, UsbSerialPort port) {
        sPort = port;
        final Intent intent = new Intent(context, SerialConsoleActivity.class);
        intent.addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP | Intent.FLAG_ACTIVITY_NO_HISTORY);
        context.startActivity(intent);
    }
}
