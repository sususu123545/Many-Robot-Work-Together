using System;
using System.Drawing;
using System.IO;
using System.Net;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

public sealed class RobotCameraViewer : Form
{
    private readonly string streamUrl;
    private readonly PictureBox picture;
    private readonly Label status;
    private CancellationTokenSource reader;

    public RobotCameraViewer(string url)
    {
        streamUrl = url;
        Text = "\u5C0F\u8F66\u6444\u50CF\u5934 /rear_camera/image_raw";
        Width = 1000;
        Height = 760;
        StartPosition = FormStartPosition.CenterScreen;
        BackColor = Color.FromArgb(24, 24, 24);

        var bar = new Panel { Dock = DockStyle.Top, Height = 46, BackColor = Color.FromArgb(42, 45, 51) };
        status = new Label { Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft, ForeColor = Color.Gold, Padding = new Padding(12, 0, 0, 0), Text = "\u6B63\u5728\u8FDE\u63A5\u6444\u50CF\u5934..." };
        var reconnect = new Button { Dock = DockStyle.Right, Width = 120, Text = "\u91CD\u65B0\u8FDE\u63A5" };
        reconnect.Click += (s, e) => StartReader();
        bar.Controls.Add(status);
        bar.Controls.Add(reconnect);

        picture = new PictureBox { Dock = DockStyle.Fill, BackColor = Color.Black, SizeMode = PictureBoxSizeMode.Zoom };
        Controls.Add(picture);
        Controls.Add(bar);

        Shown += (s, e) => StartReader();
        FormClosed += (s, e) => {
            if (reader != null) reader.Cancel();
            if (picture.Image != null) { picture.Image.Dispose(); picture.Image = null; }
        };
    }

    public static void Run(string url) { Application.Run(new RobotCameraViewer(url)); }

    private void StartReader()
    {
        if (reader != null) reader.Cancel();
        reader = new CancellationTokenSource();
        CancellationToken token = reader.Token;
        SetStatus("\u6B63\u5728\u8FDE\u63A5\u6444\u50CF\u5934...", false);
        Task.Run(() => ReadLoop(token));
    }

    private void ReadLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                var request = (HttpWebRequest)WebRequest.Create(streamUrl + "&_=" + DateTime.UtcNow.Ticks);
                request.Proxy = null;
                request.Timeout = 8000;
                request.ReadWriteTimeout = 15000;
                request.KeepAlive = true;
                request.UserAgent = "RobotCameraViewer/1.0";
                using (var response = (HttpWebResponse)request.GetResponse())
                using (var input = response.GetResponseStream())
                using (var frame = new MemoryStream())
                {
                    bool inFrame = false;
                    bool previousWasFF = false;
                    while (!token.IsCancellationRequested)
                    {
                        int value = input.ReadByte();
                        if (value < 0) break;
                        if (!inFrame)
                        {
                            if (previousWasFF && value == 0xD8)
                            {
                                frame.WriteByte(0xFF);
                                frame.WriteByte(0xD8);
                                inFrame = true;
                                previousWasFF = false;
                            }
                            else previousWasFF = value == 0xFF;
                            continue;
                        }

                        frame.WriteByte((byte)value);
                        if (previousWasFF && value == 0xD9)
                        {
                            frame.Position = 0;
                            Bitmap image;
                            using (var decoded = new Bitmap(frame)) image = new Bitmap(decoded);
                            PublishFrame(image);
                            frame.SetLength(0);
                            frame.Position = 0;
                            inFrame = false;
                            previousWasFF = false;
                        }
                        else previousWasFF = value == 0xFF;
                    }
                }
            }
            catch (Exception ex)
            {
                SetStatus("\u7B49\u5F85\u6444\u50CF\u5934\u91CD\u8FDE: " + ex.Message, false);
            }
            if (!token.IsCancellationRequested) token.WaitHandle.WaitOne(1200);
        }
    }

    private void PublishFrame(Bitmap image)
    {
        if (IsDisposed || !IsHandleCreated) { image.Dispose(); return; }
        try
        {
            BeginInvoke((Action)(() => {
                if (IsDisposed) { image.Dispose(); return; }
                Image previous = picture.Image;
                picture.Image = image;
                if (previous != null) previous.Dispose();
                status.ForeColor = Color.LightGreen;
                status.Text = "\u6444\u50CF\u5934\u5DF2\u8FDE\u63A5  " + image.Width + " x " + image.Height;
            }));
        }
        catch { image.Dispose(); }
    }

    private void SetStatus(string text, bool connected)
    {
        if (IsDisposed || !IsHandleCreated) return;
        try { BeginInvoke((Action)(() => { status.Text = text; status.ForeColor = connected ? Color.LightGreen : Color.Gold; })); }
        catch { }
    }
}
