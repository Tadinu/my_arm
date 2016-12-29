// OMNY - qmlcommonutil.js
.pragma library

.import "qrc:///javascript/qmlcommonresource.js" as GEORC // This file is the same as in K3DStudio, therefore use the same name as 'GEORC'
.import "qrc:///../K3DStudio/javascript/vendor/randomColor.js" as RANDOMCOLOR

// UTIL FUNCTS ==================================================================
var UTIL = {
    qmltypeof : function(obj, className) { // QtObject, string -> bool
        // className plus "(" is the class instance without modification
        // className plus "_QML" is the class instance with user-defined properties
        var str = obj.toString();
        return str.indexOf(className + "(") === 0 || str.indexOf(className + "_QML") === 0;
    },

    max: function(val1, val2) {
        return val1 > val2 ? val1: val2;
    },

    min: function(val1, val2) {
        return val1 < val2 ? val1: val2;
    },

    durationMSec2Time: function(duration) {
        var msec        = duration % 1000;
        var totalSec    = duration / 1000;
        var sec         = totalSec % 60;
        var totalMinute = totalSec / 60;
        var min         = totalMinute % 60;
        var hour        = totalMinute / 60;

        return Math.round(hour) + ':' + Math.round(min) + ':' + Math.round(sec) + ':' + msec;
    },

    durationMinute2Time: function(duration) {
        var min         = duration % 60;
        var hour        = duration / 60;

        return Math.round(hour) + ':' + min;
    },

    inchToMM: function(value, dimension) {
        if(dimension === undefined)
            dimension = 1;
        return Math.pow(GEORC.VAL.INCHTOMM, dimension) * value;
    },

    mmToInch: function(value, dimension) {
        if(dimension === undefined)
            dimension = 1;
        return Math.pow(GEORC.VAL.MMTOINCH, dimension) * value;
    },

    randomColor: function(options) {
        return RANDOMCOLOR.randomColor(options);
    },

    removeDuplicates: function(array) {
        var i = array.length - 1; // Start from last
        var item;
        var index;
        while(i >= 0) {
            item  = array[i];
            index = array.indexOf(item);
            //
            if(index != -1 &&
               index != i) { // Search from the beginning
                array.splice(i, 1);
            }
            i--;
        }
    },

    indexOf: function(array, value) {
        // !NOTE: HERE, WE MUST USE '==' instead of '===', and also cannot use array.indexOf() or array.lastIndexOf()
        for(var i = 0 ; i < array.length; i++) {
            //print('COMPARE', array[i], value, array[i] == value);
            if(array[i] == value)
                return i;
        }

        return -1;
    }
};

// Closure
(function() {
  /**
   * Decimal adjustment of a number.
   *
   * @param {String}  type  The type of adjustment.
   * @param {Number}  value The number.
   * @param {Integer} exp   The exponent (the 10 logarithm of the adjustment base).
   * @returns {Number} The adjusted value.
   */
  function decimalAdjust(type, value, exp) {
    // If the exp is undefined or zero...
    if (typeof exp === 'undefined' || +exp === 0) {
      return Math[type](value);
    }
    value = +value;
    exp = +exp;
    // If the value is not a number or the exp is not an integer...
    if (isNaN(value) || !(typeof exp === 'number' && exp % 1 === 0)) {
      return NaN;
    }
    // Shift
    value = value.toString().split('e');
    value = Math[type](+(value[0] + 'e' + (value[1] ? (+value[1] - exp) : -exp)));
    // Shift back
    value = value.toString().split('e');
    return +(value[0] + 'e' + (value[1] ? (+value[1] + exp) : exp));
  }

  // Decimal round
  if (!Math.round10) {
    Math.round10 = function(value, exp) {
      return decimalAdjust('round', value, exp);
    };
  }
  // Decimal floor
  if (!Math.floor10) {
    Math.floor10 = function(value, exp) {
      return decimalAdjust('floor', value, exp);
    };
  }
  // Decimal ceil
  if (!Math.ceil10) {
    Math.ceil10 = function(value, exp) {
      return decimalAdjust('ceil', value, exp);
    };
  }
})();

// TEXT FUNCTS ==================================================================
var TEXT = {

    // [ Check String content just only space symbols]
    isNullOrEmpty : function (string){
        return !string.trim(); // string.trim().length === 0
    },

    isEmpty : function (string) {
        return string.trim() === "";
    },

    // [CONTEXT MENU ITEM]
    getContextMenuItemText : function (itemNamePath) {
        return itemNamePath.split('\\').pop().split('/').pop();

        /* !NOTE: baseName if added into Extended Menu List -> is already treated in K3DItemList.addExtendedItem():
        return (baseName.length > GEORC.TEXT.ITEM_MAX_LEN)?
                baseName.substr(0, GEORC.TEXT.ITEM_MAX_LEN) + "..." + baseName.slice(extItemText.lastIndexOf('.'), baseName.length); // ".STL"
                baseName;
        */
        /*
        fileName = ‘file.jpeg’;
        fileExtension = fileName.split('.').pop();
        fileBaseName  = fileName.substr(0, fileName.lastIndexOf('.'));
        */
    },

    getContextMenuItemShrinkedText : function (itemText) {
        if (itemText.length > GEORC.TEXT.ITEM_MAX_LEN) {

            itemText   = itemText.substr(0, GEORC.TEXT.ITEM_MAX_LEN) + "...";
        }
        return itemText;
    },

    cutTextItemLastElement : function (itemText, maxLength){
        if(GEORC.CONST_STRING.indexOf(itemText) !== -1)
            return itemText;

        if(maxLength === undefined)
            maxLength = GEORC.SIZE.ITEM_TEXT_MAX_LENGTH + 3;
        var itemName;
        var itemFormat;

        var itemLastElements = itemText.slice(itemText.lastIndexOf('.'), itemText.length);

        // Last some single elements is format of some Obj/Proj
        if (GEORC.PROJ_OBJ_FORMAT.indexOf(itemLastElements) !== -1){
            itemName = itemText.slice(0, itemText.lastIndexOf('.'));
            itemFormat = itemText.slice(itemText.lastIndexOf('.'), itemText.length);
        }
        // ItemText just have name Object ( not include '.'), not is format of Obj/Proj
        // IF '.' doesn't exit, lastIndexOf('.') = 1;
        else{
            itemName = itemText;
            itemFormat = "";
        }

        var itemNameLength = itemName.length;
        if(itemNameLength > 4) {
            if(itemNameLength > maxLength)
                return itemName.substr(0, maxLength - 3) + "..." + itemFormat;
            else
                return itemName.substr(0, itemNameLength - 4) + "..." + itemFormat;
        }
        else {
            if( itemNameLength === 4 && itemName.slice(itemText.length - 3, itemText.length) === "...")
                return itemName.substr(0,1) + itemFormat;
            else {// ex: "WWWW" -> "WWW"
                if(itemNameLength > 1) {
                    return itemName.substr(0, itemNameLength - 1) + itemFormat;
                }
                else {
                    return itemText;
                }
            }
        }
    },

    cutTextItemLastElementNoDot : function (itemText, maxLength) {
        if(maxLength === undefined)
            maxLength = GEORC.SIZE.ITEM_TEXT_MAX_LENGTH;

        if(itemText.length > 1) { // > 1
            if(itemText.length > maxLength)
                itemText = itemText.substr(0, maxLength);
            else
                itemText = itemText.substr(0, itemText.length - 1);
        }
        else {
            itemText = itemText.substr(0,1);
        }
        return itemText;
    },


    getContextMenuItemShrinkedTextII : function (itemText, maxlength) {
        if (itemText.length > maxlength) {
            itemText   = "..." + itemText.substr(itemText.length - GEORC.TEXT.ITEM_MAX_LEN, GEORC.TEXT.ITEM_MAX_LEN);
        }

        return itemText;
    },

    // [MACHINE BAR ITEM]
    getItemTextSetMaxLength : function (itemText, maxLength) {
        if (itemText.length > maxLength) {
            itemText = itemText.substr(0, maxLength -2) + "...";
        }
        // Treat itemText
        return itemText;
    },
};

var FLAG = {
    test: function (value, flag) {
        return ((value & flag) === flag);
    }
};
