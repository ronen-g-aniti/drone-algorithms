using System;
using System.Data;
using System.Data.SQLite;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;

namespace telemetry_data_processor
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            InitializeChart();
        }

        private void InitializeChart() // Initialize the chart, which will display telemetry data
        {
            ChartArea chartArea = new ChartArea(); // Create a new chart area

            chartArea.AxisX.Title = "Timestep";
            chartArea.AxisX.TitleFont = new Font("Arial", 12, FontStyle.Bold);
            chartArea.AxisY.Title = "Speed (m/s)";
            chartArea.AxisY.TitleFont = new Font("Arial", 12, FontStyle.Bold);
            chartTelemetry.ChartAreas.Clear();
            chartTelemetry.ChartAreas.Add(chartArea); // Add the chart area to the chart 
            Title chartTitle = new Title() // Create a new title for the chart
            {
                Text = "Telemetry Data",
                Font = new Font("Arial", 16, FontStyle.Bold),
                ForeColor = Color.Blue
            };
            chartTelemetry.Titles.Add(chartTitle); // Add the title to the chart
            InitializeChartSeries(); // Initialize the chart series
        }

        private void InitializeChartSeries()
        {
            Series speedSeries = new Series("Speed") // Create a new series for speed data
            {
                ChartType = SeriesChartType.Line,
                XValueMember = "id",
                YValueMembers = "speed"
            };
            chartTelemetry.Series.Add(speedSeries); // Add the speed series to the chart
        }

        private void btnLoadData_Click(object sender, EventArgs e) // Load telemetry data when the button is clicked
        {
            LoadTelemetryData();
        }

        private void LoadTelemetryData() // Load telemetry data from the database
        { 
            string connectionString = "Data Source=C:\\Users\\Ronen\\Desktop\\drone-algorithms\\telemetry_data_processor\\telemetry_data_processor\\telemetry_data_processor\\telemetry_data.db;Version=3;"; // Connection string for the SQLite database
            using (SQLiteConnection conn = new SQLiteConnection(connectionString)) // Create a new SQLite connection
            {
                conn.Open(); // Open the connection, which will be automatically closed when the using block is exited
                string query = "SELECT id, speed FROM telemetry"; // Query to select all data from the telemetry table
                using (SQLiteDataAdapter adapter = new SQLiteDataAdapter(query, conn)) // Create a new data adapter,
                                                                                       // which serves the purpose of a bridge between the
                                                                                       // database and the data being displayed
                {
                    DataTable dataTable = new DataTable(); // Create a new data table to store the telemetry data
                    adapter.Fill(dataTable); // Fill the data table with the data from the database so that it can be displayed

                    chartTelemetry.DataSource = dataTable; // Set the data source for the chart so that it knows where to get the data from
                    chartTelemetry.DataBind(); // Bind the data to the chart so that the data is displayed

                }
            }
        }
    }
}
