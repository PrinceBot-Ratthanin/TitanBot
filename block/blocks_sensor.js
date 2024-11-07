const dirIcon = Vue.prototype.$global.board.board_info.dir;
module.exports = function(Blockly){
  'use strict';

Blockly.Blocks['sw1_press'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Wait SW1 pressed");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("Buzzer beep");
 this.setHelpUrl("");
  }
};
Blockly.Blocks['bootel_press'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Wait Boot button pressed");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("Buzzer beep");
 this.setHelpUrl("");
  }
};
Blockly.Blocks['button_1_status'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldImage("/static/icons/sw12x12.png", 20, 20, "*"))
        .appendField("SW1(IO6) Status ");
    this.setInputsInline(true);
    this.setOutput(true, "Number");
    this.setColour(65);
 this.setTooltip("get SW1 pressed or not");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['Ultrasinoc_sensor'] = {
  init: function() {
    this.appendDummyInput()
        //.appendField(new Blockly.FieldImage("/static/icons/bmx055.png", 20, 20, "*"))
        .appendField("Read Distance");
    this.setInputsInline(true);
    this.setOutput(true, "Number");
    this.setColour(65);
 this.setTooltip("Read Distance");
 this.setHelpUrl("");
  }
};
Blockly.Blocks['Light_Sensor'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Front Light sensor(ADC):")
        .appendField(new Blockly.FieldDropdown([["0","0"],["1","1"], ["2","2"], ["3","3"],
        										["4","4"],["5","5"], ["6","6"], ["7","7"]]), "pin");
    this.setOutput(true);
    this.setColour(65);
    this.setTooltip("read pos EncoderA Pin 26");
    this.setHelpUrl("");
  }
};
Blockly.Blocks['Light_Sensor_B'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Back Light sensor(ADC):")
        .appendField(new Blockly.FieldDropdown([["0","0"],["1","1"], ["2","2"], ["3","3"],
        										["4","4"],["5","5"], ["6","6"], ["7","7"]]), "pin");
    this.setOutput(true);
    this.setColour(65);
    this.setTooltip("read pos EncoderA Pin 26");
    this.setHelpUrl("");
  }
};
Blockly.Blocks['Light_Sensor_C'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Center Light sensor(ADC):")
        .appendField(new Blockly.FieldDropdown([["0","0"],["1","1"], ["2","2"], ["3","3"],
        										["4","4"],["5","5"], ["6","6"], ["7","7"]]), "pin");
    this.setOutput(true);
    this.setColour(65);
    this.setTooltip("read pos EncoderA Pin 26");
    this.setHelpUrl("");
  }
};
Blockly.Blocks['INPUT_digital'] = {
  init: function() {
    // this.appendDummyInput()
    //     //.appendField(new Blockly.FieldImage("/static/icons/bmx055.png", 20, 20, "*"))
    //     .appendField("Read digital status");
    this.appendValueInput("pin")
      .appendField("Read digital status at");
    this.setInputsInline(true);
    this.setOutput(true, "Number");
    this.setColour(65);
 this.setTooltip("Read digital status at");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['Puppy_beep'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Buzzer beep");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("Buzzer beep");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['TCS_color_status'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("TCS_color at")
          .appendField(new Blockly.FieldDropdown([["R","0"],["G","1"], ["B","2"]]), "_color");
      this.setInputsInline(true);
      this.setOutput(true, "Number");
      this.setColour(65);
   this.setTooltip("color");
   this.setHelpUrl("");
    }
  };

Blockly.Blocks['TCS_color_config'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("TCS_color_2  Config Pin SDA")
        .appendField(new Blockly.FieldDropdown([["S1","12"],["S2","13"], ["S3","14"], ["S4","15"],
                            ["D1","25"],["D2","27"], ["D3","28"], ["D4","29"]]), "pin_SDA");
    this.appendDummyInput()
        .appendField("Pin SCL")
        .appendField(new Blockly.FieldDropdown([["S2","13"],["S1","12"], ["S3","14"], ["S4","15"],
                            ["D1","25"],["D2","27"], ["D3","28"], ["D4","29"]]), "pin_SCL");
     this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
    this.setTooltip("config pin for TCS34725");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['TCS_color_status_B'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("TCS_color_2 at")
          .appendField(new Blockly.FieldDropdown([["R","0"],["G","1"], ["B","2"]]), "_color");
      this.setInputsInline(true);
      this.setOutput(true, "Number");
      this.setColour(65);
   this.setTooltip("color");
   this.setHelpUrl("");
    }
  };


}