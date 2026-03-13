
# Data Warehousing Project – ABC Mining

## Overview
This project demonstrates the design and implementation of a **data warehouse** for a fictional mining company called **ABC Mining**. The company operates multiple iron ore mines in Western Australia and requires improved analytics for operational efficiency, supply chain management, and sales forecasting.

The warehouse integrates operational data sources and models them into dimensional data cubes that support business intelligence and management reporting.

## Objectives
The main goals of the project were:

- Design a dimensional data warehouse schema
- Model key business processes using star schemas
- Create data cubes for analytical queries
- Simulate operational data to populate the warehouse
- Demonstrate how business intelligence queries can support management decisions

## Business Context
ABC Mining extracts iron ore from open‑cut mines and transports it via rail to Port Hedland for shipment to international customers.

Profit is determined by:

Profit = Revenue – Operational Costs

The data warehouse focuses on helping management:

- Reduce operational costs
- Improve supply chain visibility
- Forecast demand from long‑term supply contracts
- Analyse production efficiency across mines and equipment

## Data Warehouse Architecture

Operational Systems → ETL / Simulation → Data Warehouse → BI Analysis

For this project, operational data was **simulated using Python** and loaded directly into the data warehouse schema to simplify development.

## Data Cubes

### 1. Truck Management Data Cube
Tracks the performance and cost of mining trucks.

**Key Dimensions**
- Date
- Time
- Truck
- Driver
- Location (Mine)
- Mission

**Key Facts**
- Fuel consumption
- Distance travelled
- Ore delivered (tonnes)
- Parts and labour costs
- Overspeed / brake / overload events
- Accidents and tyre failures

**Example Analytics**
- Cost per tonne of ore delivered
- Human vs autonomous truck performance
- Event frequency by driver demographics
- Productivity by mine and truck type

---

### 2. Stockpile & Supply Chain Data Cube
Provides visibility into ore flow from mines to port.

**Key Dimensions**
- Date
- Stockpile
- Location
- Train
- Deposit tier

**Key Facts**
- Stockpile loading levels
- Ore flow rate
- Train capacity and movement
- In‑ground resource estimates

**Example Analytics**
- Stockpile utilisation trends
- Supply bottlenecks
- Production capacity planning

---

### 3. Order Pipeline Data Cube
Tracks long‑term iron ore supply contracts and shipments.

**Key Dimensions**
- Customer
- Country
- Port
- Transporter
- Date

**Key Facts**
- Shipment quantity
- Order quantity
- Contract price

**Example Analytics**
- Demand forecast (month / year / decade)
- Revenue by region
- Customer profitability

---

## Dimensional Modeling
The warehouse follows **Kimball-style dimensional modeling** with:

- **Fact tables** storing measurable business events
- **Dimension tables** providing context for analysis
- **Star schemas** optimized for OLAP queries

Example fact table grain:

Truck cube grain: **one truck mission (round trip)**

## Data Generation
Operational datasets were generated using **Python simulations** to mimic:

- Truck telemetry and events
- Stockpile levels and ore flows
- Train movement
- Long‑term supply contracts

This allowed realistic warehouse datasets to be created without building full operational systems.

## Technologies Used

- SQL Server (data warehouse schema)
- Python (data generation simulations)
- OLAP cube design concepts
- Dimensional modeling

## Repository Contents

| File | Description |
|-----|-------------|
| project_report.pdf | Full academic project report |
| schema_diagrams/ | Star schema diagrams |
| simulation_code/ | Python scripts used to generate data |
| warehouse_schema.sql | SQL schema definitions |
| README.md | Project overview |

## Author

Nathan Scott  
