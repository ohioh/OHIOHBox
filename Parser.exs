
############################################################################################################
#
#   Author: Tjark Ziehm ( OHIOH e.V. )
#   name: decoderCovidResearchLPP.ex
#   edit: 2021-29-07
#   version: 0.9
#   questions: kontakt@ohioh.de
#   documentation: coming soon
#
############################################################################################################
  # Changelog:
  #   29.07.2021 [TZ]: created the file
  #  
  #   
  defmodule Parser do
    use Platform.Parsing.Behaviour
    
    def parse(<<state::8, bat::8, temp::8, hum::8, pm1::8, pm25::8, pm4::8, pm5::8, pm10::8, voc::8, co2::8>>, %{meta: %{frame_port: 2}}) do
      %{
        message_type: "OHIOH_Research_Data",
        status: state,
        battery: bat,
        temperature: temp,
        humidity: hum,
        feinstaub1: pm1,
        feinstaub25: pm25,
        feinstaub4: pm4,
        feinstaub5: pm5,
        feinstaub10: pm10,
        loesemittel: voc,
        codioxid: co2
      }
    end
    
    def fields do
      [
        %{
          "field" => "status",
        "display" => "MessageType"
        },
        %{
          "field" => "battery",
        "display" => "Battery",
        "unit" => "%"
        },
        %{
          "field" => "temperature",
        "display" => "Temp",
        "unit" => "°C"
        },
        %{
          "field" => "humidity",
        "display" => "Luftfeuchtigkeit",
        "unit" => "%"
        },
        %{
          "field" => "feinstaub1",
        "display" => "Feinstaub PM1",
        "unit" => "ppm"
        },
        %{
          "field" => "feinstaub25",
        "display" => "Feinstaub PM2.5",
        "unit" => "ppm"
        },
        %{
          "field" => "feinstaub4",
        "display" => "Feinstaub PM4",
        "unit" => "ppm"
        },
        %{
          "field" => "feinstaub5",
        "display" => "Feinstaub PM5",
        "unit" => "ppm"
        },
        %{
          "field" => "feinstaub10",
        "display" => "Feinstaub PM10",
        "unit" => "ppm"
        },
        %{
          "field" => "loesemittel",
        "display" => "VOC",
        "unit" => "gr."
        },
        %{
          "field" => "codioxid",
        "display" => "C0²",
        "unit" => "gr."
        }
        
      ]  
    end
    
  end
