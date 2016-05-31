package com.example.terence.androidcamera;

// libraries
import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Bundle;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.TextView;
import java.io.IOException;
import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;

import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;

public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640,480,Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;
    static long prevtime = 0; // for FPS calculation

    // array that stores the colors to draw
    int[] thresholdedColors1 = new int[bmp.getWidth()];
    int[] thresholdedColors2 = new int[bmp.getWidth()];

    SeekBar myControl;
    TextView myTextView;
    SeekBar myControl2;
    TextView myTextView2;

    // establish progress changes as a global variable
    int progressChanged = 0;
    int progressChanged2 = 0;
    int thresholdValue1 = 0;
    int thresholdValue2 = 0;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off

        mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
        mSurfaceHolder = mSurfaceView.getHolder();

        mTextureView = (TextureView) findViewById(R.id.textureview);
        mTextureView.setSurfaceTextureListener(this);

        mTextView = (TextView) findViewById(R.id.cameraStatus);

        paint1.setColor(0xffff0000); // red
        paint1.setTextSize(24);

        // instantiate the seekbar and textview
        myControl = (SeekBar) findViewById(R.id.seek1);
        myTextView = (TextView) findViewById(R.id.textView01);
        myTextView.setText("Enter whatever you Like!");
        setMyControlListener();
        // Seekbar2 and Textview2
        myControl2 = (SeekBar) findViewById(R.id.seek2);
        myTextView2 = (TextView) findViewById(R.id.textView02);
        myTextView2.setText("Enter whatever you Like!");
        setMyControlListener2();

    }

    private void setMyControlListener() {
        myControl.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
                thresholdValue1 = progressChanged*10;
                myTextView.setText("The value is: "+progress+ "."+ "The threshold value is: " +thresholdValue1);
            }
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

    private void setMyControlListener2() {
        myControl2.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged2 = progress;
                thresholdValue2 = progressChanged2*10;
                myTextView2.setText("The value is: "+progress+ "."+ "The threshold value is: " +thresholdValue2);
            }
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setColorEffect(Camera.Parameters.EFFECT_MONO); // black and white
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
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

            int[] pixels1 = new int[bmp.getWidth()];
            int[] pixels2 = new int[bmp.getWidth()];
            int startY1 = 15; // which row in the bitmap to analyse to read
            int startY2 = 300; // which row in the bitmap to analyse to read

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
            int COM1 = 0;
            int COM2 = 0;

            for (int i = 0; i < bmp.getWidth(); i++) {
                // sum the red, green and blue, subtract from 255 to get the darkness of the pixel.
                // if it is greater than some value (600 here), consider it black
                // play with the 600 value if you are having issues reliably seeing the line
                if (255*3-(red(pixels1[i])+green(pixels1[i])+blue(pixels1[i])) > thresholdValue1) {
//                if (255*3-(red(pixels1[i])+green(pixels1[i])+blue(pixels1[i])) > 600) {
                    thresholdedPixels1[i] = 255*3;
                    thresholdedColors1[i] = rgb(0, 255, 0);
                }
                else {
                    thresholdedPixels1[i] = 0;
                    thresholdedColors1[i] = rgb(0, 0, 0);
                }
                wbTotal1 = wbTotal1 + thresholdedPixels1[i];
                wbCOM1 = wbCOM1 + thresholdedPixels1[i]*i;
            }

            for (int i = 0; i < bmp.getWidth(); i++) {
                // sum the red, green and blue, subtract from 255 to get the darkness of the pixel.
                // if it is greater than some value (600 here), consider it black
                // play with the 600 value if you are having issues reliably seeing the line
                if (255*3-(red(pixels2[i])+green(pixels2[i])+blue(pixels2[i])) > thresholdValue2) {
//                if (255*3-(red(pixels2[i])+green(pixels2[i])+blue(pixels2[i])) > 600) {
                    thresholdedPixels2[i] = 255*3;
                    thresholdedColors2[i] = rgb(0, 255, 0);
                }
                else {
                    thresholdedPixels2[i] = 0;
                    thresholdedColors2[i] = rgb(0, 0, 0);
                }
                wbTotal2 = wbTotal2 + thresholdedPixels2[i];
                wbCOM2 = wbCOM2 + thresholdedPixels2[i]*i;
            }

            bmp.setPixels(thresholdedColors1, 0, bmp.getWidth(), 0, startY1, bmp.getWidth(), 1);
            bmp.setPixels(thresholdedColors2, 0, bmp.getWidth(), 0, startY2, bmp.getWidth(), 1);

            //watch out for divide by 0
            if (wbTotal1<=0) {
                COM1 = bmp.getWidth()/2;
            }
            else {
                COM1 = wbCOM1/wbTotal1;
            }

            //watch out for divide by 0
            if (wbTotal2<=0) {
                COM2 = bmp.getWidth()/2;
            }
            else {
                COM2 = wbCOM2/wbTotal2;
            }

            // draw a circle where you think the COM is
            canvas.drawCircle(COM1, startY1, 5, paint1);
            canvas.drawCircle(COM2, startY2, 5, paint1);

            // also write the value as text
            // max COM = 600
            canvas.drawText("COM1 = " + COM1 + " COM2 = " + COM2, 10, 200, paint1);
            c.drawBitmap(bmp, 0, 0, null);
            mSurfaceHolder.unlockCanvasAndPost(c);

            // calculate the FPS to see how fast the code is running
            long nowtime = System.currentTimeMillis();
            long diff = nowtime - prevtime;
            mTextView.setText("FPS " + 1000/diff);
            prevtime = nowtime;
        }
    }
}
