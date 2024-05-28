namespace telemetry_data_processor
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea2 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend2 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series2 = new System.Windows.Forms.DataVisualization.Charting.Series();
            btnLoadData = new Button();
            chartTelemetry = new System.Windows.Forms.DataVisualization.Charting.Chart();
            ((System.ComponentModel.ISupportInitialize)chartTelemetry).BeginInit();
            SuspendLayout();
            // 
            // btnLoadData
            // 
            btnLoadData.Location = new Point(350, 409);
            btnLoadData.Name = "btnLoadData";
            btnLoadData.Size = new Size(94, 29);
            btnLoadData.TabIndex = 0;
            btnLoadData.Text = "Load Data";
            btnLoadData.UseVisualStyleBackColor = true;
            btnLoadData.Click += new System.EventHandler(this.btnLoadData_Click); // Add an event handler for the button click;
                                                                                  // The reason the += operator is used is
                                                                                  // because there can be multiple event handlers
                                                                                  // for a single event. In this case, the event
                                                                                  // handler is the btnLoadData_Click method.
            // 
            // chartTelemetry
            // 
            chartArea2.Name = "ChartArea1";
            chartTelemetry.ChartAreas.Add(chartArea2);
            legend2.Name = "Legend1";
            chartTelemetry.Legends.Add(legend2);
            chartTelemetry.Location = new Point(12, 12);
            chartTelemetry.Name = "chartTelemetry";
            series2.ChartArea = "ChartArea1";
            series2.Legend = "Legend1";
            series2.Name = "Series1";
            chartTelemetry.Series.Add(series2);
            chartTelemetry.Size = new Size(776, 375);
            chartTelemetry.TabIndex = 1;
            chartTelemetry.Text = "chart1";
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(8F, 20F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(800, 450);
            Controls.Add(chartTelemetry);
            Controls.Add(btnLoadData);
            Name = "Form1";
            Text = "c";
            ((System.ComponentModel.ISupportInitialize)chartTelemetry).EndInit();
            ResumeLayout(false);
        }

        #endregion

        private Button btnLoadData;
        private System.Windows.Forms.DataVisualization.Charting.Chart chartTelemetry;
    }
}
